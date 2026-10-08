/*
 * =====================================================================
 *  Projet Objets Communicants - ENSIM
 *  Noeud RELAIS (ID 100) - Arthur & Lucas
 *  Materiel : Arduino Uno + shield XBee + XBee Pro (802.15.4)
 * =====================================================================
 *
 *  Role : faire passer les trames entre le Hub (000), hors de portee,
 *         et les noeuds qui ne l'atteignent pas directement :
 *         temperature interieure (010), servomoteur (011) et humidite (110).
 *         La liste est tenue par noeudDerriereRelais().
 *
 *  Trame (4 octets) :
 *    [0] 0xAA                          synchro
 *    [1] dest(3) | exp(3) | d9 d8      en-tete
 *    [2] d7 ... d0                     donnees
 *    [3] checksum = (octets 0 a 2) % 256  (voir CHECKSUM_MODULO)
 *
 *  Routage :
 *    exp = Hub  et dest derriere le relais  ->  envoyee au noeud dest
 *    dest = Hub et exp derriere le relais   ->  envoyee au Hub
 *    tout le reste                          ->  ignoree
 *
 *  La trame est retransmise sans modification (checksum inchange).
 * =====================================================================
 */

#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================ CONFIGURATION ============================

// true = le relais envoie en boucle les trames de test au servomoteur (testActionneur)
const bool debug = false;
const unsigned long DELAI_TEST_MS = 2000;

// Cablage du shield :
//   1 = XBee sur pins 2/3 (SoftwareSerial), moniteur seriea USB dispo pour le debug
//   0 = XBee sur le Serial materiel (pins 0/1), pas de debug possible
#define XBEE_SUR_SOFTSERIAL 1
const uint8_t PIN_XBEE_RX = 2;   // relie a DOUT du XBee
const uint8_t PIN_XBEE_TX = 3;   // relie a DIN du XBee
const long    BAUD_XBEE   = 9600;
const long    BAUD_DEBUG  = 115200;

// Mode d'envoi :
//   1 = unicast   : le relais change sa destination radio (ATDL) avant chaque envoi
//   0 = broadcast : DL = 0xFFFF, tout le monde recoit, le tri se fait sur le champ dest
#define MODE_UNICAST 1

// Checksum : A ALIGNER AVEC TOUTE LA CLASSE
#define CHECKSUM_AVEC_SYNC 1          // 1 = la somme inclut l'octet 0xAA (octets 1 a 3 du tableau)
// 256 = somme tronquee sur 8 bits. C'est ce que font les noeuds observes sur le reseau
//       (trame AA 88 1B 4D : AA+88+1B = 333, et 333 % 256 = 0x4D).
// 255 = modulo 255, comme ecrit dans le sujet. Les deux ne divergent que si la somme
//       depasse 255, d'ou des rejets intermittents si tout le monde n'a pas la meme valeur.
const uint16_t CHECKSUM_MODULO = 256;

// Reseau (tableau : canal C, PAN ID 1234)
const char RADIO_CANAL[]  = "C";
const char RADIO_PAN_ID[] = "1234";

// IDs applicatifs (tableau)
const uint8_t ID_HUB      = 0b000;
const uint8_t ID_PHOTO    = 0b001;
const uint8_t ID_TEMP_INT = 0b010;
const uint8_t ID_SERVO    = 0b011;
const uint8_t ID_RELAIS   = 0b100;
const uint8_t ID_TEMP_EXT = 0b101;
const uint8_t ID_HUMIDITE = 0b110;

// Adresses radio XBee (parametre MY de chaque module), indexees par ID applicatif.
// Convention proposee : MY = 0x10 + ID. On evite 0x0000, valeur usine de tous les modules.
// A VALIDER AVEC LES AUTRES GROUPES.
const uint16_t MY_XBEE[8] = {
  0x0010,  // 000 Hub
  0x0011,  // 001 Photoresistance
  0x0012,  // 010 Temperature interieure
  0x0013,  // 011 Servomoteur
  0x0014,  // 100 Relais (nous)
  0x0015,  // 101 Temperature + humidite exterieure
  0x0016,  // 110 Humidite            <- A CONFIRMER avec le groupe concerne
  0xFFFF   // 111 inutilise
};

// Temporisations
const unsigned int  GT_USINE_MS      = 1000;  // guard time par defaut du XBee
const unsigned int  GT_RAPIDE_MS     = 50;    // guard time regle au demarrage (ATGT 32)
const unsigned long TIMEOUT_TRAME_MS = 100;   // une trame commencee doit finir avant ce delai
const unsigned long DUREE_LED_MS     = 60;
const unsigned long PERIODE_BILAN_MS = 10000;

const uint8_t SYNC = 0xAA;

// Ecran LCD 16x2 en I2C (SDA = A4, SCL = A5 sur Uno)
const uint8_t LCD_ADRESSE_I2C = 0x27;
const uint8_t LCD_COLONNES    = 16;
const uint8_t LCD_LIGNES      = 2;

// Noms d'au plus 6 caracteres pour tenir "exp > dest" sur une ligne de 16.
const char NOM_COURT[8][7] = {
  "Hub", "Photo", "TmpInt", "Servo", "Relais", "TmpExt", "Humid", "?"
};

// ============================ SERIES / DEBUG ============================

#if XBEE_SUR_SOFTSERIAL
SoftwareSerial xbee(PIN_XBEE_RX, PIN_XBEE_TX);
#define DEBUG(...)   Serial.print(__VA_ARGS__)
#define DEBUGLN(...) Serial.println(__VA_ARGS__)
#else
#define xbee Serial
#define DEBUG(...)
#define DEBUGLN(...)
#endif

LiquidCrystal_I2C lcd(LCD_ADRESSE_I2C, LCD_COLONNES, LCD_LIGNES);

// ================================ ETAT =================================

uint8_t       fenetre[4];             // octets en cours de reception
uint8_t       nbOctets       = 0;
unsigned long dernierOctetMs = 0;

uint16_t      dlCourant = 0;          // destination radio actuellement reglee
bool          dlConnu   = false;

unsigned long nbRelayees = 0, nbRejetees = 0, nbIgnorees = 0;
unsigned long ledAllumeeMs = 0, dernierBilanMs = 0;

// ============================ OUTILS TRAME =============================

uint8_t lireDest(const uint8_t* t)  { return (t[1] >> 5) & 0x07; }
uint8_t lireExp(const uint8_t* t)   { return (t[1] >> 2) & 0x07; }
uint16_t lireData(const uint8_t* t) { return ((uint16_t)(t[1] & 0x03) << 8) | t[2]; }

uint8_t calculerChecksum(const uint8_t* t) {
  uint16_t somme = (uint16_t)t[1] + t[2];
#if CHECKSUM_AVEC_SYNC
  somme += t[0];
#endif
  return somme % CHECKSUM_MODULO;
}

// Meme somme, mais avec l'autre convention de modulo : sert uniquement au diagnostic.
uint8_t checksumAutreModulo(const uint8_t* t) {
  uint16_t somme = (uint16_t)t[1] + t[2];
#if CHECKSUM_AVEC_SYNC
  somme += t[0];
#endif
  return somme % (CHECKSUM_MODULO == 255 ? 256 : 255);
}

void afficherOctet(uint8_t b) {
  if (b < 0x10) DEBUG('0');
  DEBUG(b, HEX);
}

void afficherId(uint8_t id) {
  for (int8_t i = 2; i >= 0; i--) DEBUG((id >> i) & 1);
}

// Nom lisible du noeud, pour les affichages en clair.
const __FlashStringHelper* nomNoeud(uint8_t id) {
  switch (id) {
    case ID_HUB:      return F("Hub");
    case ID_PHOTO:    return F("Photoresistance");
    case ID_TEMP_INT: return F("Temperature interieure");
    case ID_SERVO:    return F("Servomoteur");
    case ID_RELAIS:   return F("Relais");
    case ID_TEMP_EXT: return F("Temp/humidite exterieure");
    case ID_HUMIDITE: return F("Humidite");
    default:          return F("inconnu");
  }
}

// Adresse radio (parametre MY) associee a un ID applicatif : "0x0012".
// En mode transparent le XBee ne nous transmet pas l'adresse source du paquet,
// on ne peut donc afficher que l'adresse *attendue* d'apres le champ exp.
void afficherAdresse(uint16_t adresse) {
  DEBUG(F("0x"));
  for (int8_t i = 12; i >= 0; i -= 4) {
    uint8_t quartet = (adresse >> i) & 0x0F;
    DEBUG(quartet, HEX);
  }
}

// "Hub (000)" ou, avec adresse=true, "Hub (000), MY attendu 0x0010"
void afficherNoeud(uint8_t id, bool adresse = false) {
  DEBUG(nomNoeud(id));
  DEBUG(F(" ("));
  afficherId(id);
  DEBUG(')');
  if (adresse) {
    DEBUG(F(", MY attendu "));
    afficherAdresse(MY_XBEE[id]);
  }
}

// Les 4 octets en hexa : "AA 4A 1F 73"
void afficherOctets(const uint8_t* t) {
  for (uint8_t i = 0; i < 4; i++) {
    if (i) DEBUG(' ');
    afficherOctet(t[i]);
  }
}

// Trame decodee en texte, sur plusieurs lignes :
//   RX trame : AA 4A 1F 73
//      de       : Hub (000), MY attendu 0x0010
//      vers     : Temperature interieure (010)
//      donnees  : 543 (0x21F)
//      checksum : 0x73 (correct)
void afficherTrame(const uint8_t* t) {
  uint16_t data = lireData(t);

  DEBUG(F("RX trame : "));
  afficherOctets(t);
  DEBUGLN();

  DEBUG(F("   de       : "));  afficherNoeud(lireExp(t), true);  DEBUGLN();
  DEBUG(F("   vers     : "));  afficherNoeud(lireDest(t)); DEBUGLN();

  DEBUG(F("   donnees  : "));  DEBUG(data);
  DEBUG(F(" (0x"));            DEBUG(data, HEX);
  DEBUGLN(')');

  DEBUG(F("   checksum : 0x")); afficherOctet(t[3]);
  DEBUGLN(calculerChecksum(t) == t[3] ? F(" (correct)") : F(" (FAUX)"));
}

// Ligne 1 : "TmpInt > Hub"   Ligne 2 : "Valeur: 24"
void afficherTrameLcd(const uint8_t* t) {
  lcd.clear();
  lcd.print(NOM_COURT[lireExp(t)]);
  lcd.print(F(" > "));
  lcd.print(NOM_COURT[lireDest(t)]);
  lcd.setCursor(0, 1);
  lcd.print(F("Valeur: "));
  lcd.print(lireData(t));
}

// ============================== RECEPTION ==============================

// Apres un checksum faux, le 0xAA de depart etait peut-etre une donnee :
// on cherche une autre synchro dans les octets deja recus au lieu de tout jeter.
void resynchroniser() {
  uint8_t i = 1;
  while (i < 4 && fenetre[i] != SYNC) i++;
  nbOctets = 4 - i;
  memmove(fenetre, fenetre + i, nbOctets);
}

// Renvoie true et remplit 'trame' quand une trame valide est recue.
bool lireTrame(uint8_t* trame) {
  if (nbOctets > 0 && millis() - dernierOctetMs > TIMEOUT_TRAME_MS) {
    nbOctets = 0;  // trame incomplete abandonnee
  }

  while (xbee.available()) {
    uint8_t b = xbee.read();
    dernierOctetMs = millis();

    if (nbOctets == 0 && b != SYNC) continue;  // on attend la synchro
    fenetre[nbOctets++] = b;

    if (nbOctets == 4) {
      if (calculerChecksum(fenetre) == fenetre[3]) {
        memcpy(trame, fenetre, 4);
        nbOctets = 0;
        return true;
      }
      nbRejetees++;
      DEBUG(F("! trame rejetee (checksum faux) : "));
      afficherOctets(fenetre);
      DEBUG(F(" -> attendu 0x"));
      afficherOctet(calculerChecksum(fenetre));
      if (fenetre[3] == checksumAutreModulo(fenetre)) {
        DEBUG(F(" (l'emetteur utilise le modulo "));
        DEBUG(CHECKSUM_MODULO == 255 ? 256 : 255);
        DEBUG(F(" : convention a aligner)"));
      }
      DEBUGLN();
      resynchroniser();
    }
  }
  return false;
}

// ============================ COMMANDES AT =============================

// Attend "OK\r" ; les autres octets recus pendant l'attente sont perdus.
bool attendreOK(unsigned long timeoutMs) {
  unsigned long debut = millis();
  uint8_t etat = 0;
  while (millis() - debut < timeoutMs) {
    if (!xbee.available()) continue;
    char c = xbee.read();
    if      (etat == 0 && c == 'O')  etat = 1;
    else if (etat == 1 && c == 'K')  etat = 2;
    else if (etat == 2 && c == '\r') return true;
    else    etat = (c == 'O') ? 1 : 0;
  }
  return false;
}

bool entrerModeCommande(unsigned int guardTimeMs) {
  delay(guardTimeMs + 20);     // silence avant "+++"
  xbee.print(F("+++"));
  return attendreOK(guardTimeMs + 1500);
}

bool envoyerAT(const __FlashStringHelper* cmd, const char* param) {
  xbee.print(cmd);
  xbee.print(param);
  xbee.print('\r');
  return attendreOK(500);
}

bool quitterModeCommande() {
  xbee.print(F("ATCN\r"));
  return attendreOK(500);
}

// Configure le XBee du relais a chaque demarrage (pas de ATWR : rien n'est ecrit en flash).
bool configurerXBee() {
  if (!entrerModeCommande(GT_USINE_MS)) return false;

  char my[5], dl[5];
  sprintf(my, "%X", (unsigned int)MY_XBEE[ID_RELAIS]);
#if MODE_UNICAST
  sprintf(dl, "%X", (unsigned int)MY_XBEE[ID_HUB]);
#else
  sprintf(dl, "FFFF");
#endif

  bool ok = true;
  ok &= envoyerAT(F("ATAP"), "0");           // mode transparent
  ok &= envoyerAT(F("ATCH"), RADIO_CANAL);
  ok &= envoyerAT(F("ATID"), RADIO_PAN_ID);
  ok &= envoyerAT(F("ATMY"), my);
  ok &= envoyerAT(F("ATDH"), "0");           // adressage 16 bits
  ok &= envoyerAT(F("ATDL"), dl);
  ok &= envoyerAT(F("ATGT"), "32");          // guard time 0x32 = 50 ms
  ok &= quitterModeCommande();

  if (ok) {
#if MODE_UNICAST
    dlCourant = MY_XBEE[ID_HUB];
#else
    dlCourant = 0xFFFF;
#endif
    dlConnu = true;
  }
  return ok;
}

// Change la destination radio, seulement si elle est differente de l'actuelle.
bool changerDestinationRadio(uint16_t adresse) {
  if (dlConnu && adresse == dlCourant) return true;
  if (!entrerModeCommande(GT_RAPIDE_MS)) return false;

  char dl[5];
  sprintf(dl, "%X", (unsigned int)adresse);
  bool ok = envoyerAT(F("ATDL"), dl);
  ok = quitterModeCommande() && ok;

  if (ok) { dlCourant = adresse; dlConnu = true; }
  else    { dlConnu = false; }
  return ok;
}

// =============================== ROUTAGE ===============================

// Noeuds hors de portee du Hub : c'est pour eux que le relais existe.
// Ajouter ou retirer un noeud ici suffit, les deux sens suivent.
bool noeudDerriereRelais(uint8_t id) {
  return id == ID_TEMP_INT    // 010, P20
      || id == ID_SERVO       // 011, P20
      || id == ID_HUMIDITE;   // 110
}

// Renvoie l'ID vers lequel relayer la trame, ou -1 pour l'ignorer.
int8_t cibleRelais(uint8_t dest, uint8_t exp) {
  if (exp == ID_HUB && noeudDerriereRelais(dest)) return dest;    // Hub -> noeud
  if (dest == ID_HUB && noeudDerriereRelais(exp)) return ID_HUB;  // noeud -> Hub
  return -1;
}

bool envoyerVers(uint8_t id, const uint8_t* trame) {
#if MODE_UNICAST
  if (!changerDestinationRadio(MY_XBEE[id])) return false;
#else
  (void)id;
#endif
  xbee.write(trame, 4);
  return true;
}

// Le checksum est calcule a l'envoi pour suivre CHECKSUM_MODULO.
const uint8_t TRAMES_TEST_SERVO[][3] = {
  {0xAA, 0x60, 0x01},  // ouvrir
  {0xAA, 0x60, 0x00},  // fermer
  {0xAA, 0x60, 0x02},  // demande d'etat
};

void testActionneur() {
  for (const uint8_t* entete : TRAMES_TEST_SERVO) {
    uint8_t trame[4] = {entete[0], entete[1], entete[2], 0};
    trame[3] = calculerChecksum(trame);
    DEBUG(F("TX test : "));
    afficherOctets(trame);
    DEBUGLN(envoyerVers(ID_SERVO, trame) ? F("") : F("  ! echec ATDL"));
    afficherTrameLcd(trame);
    delay(DELAI_TEST_MS);
  }
}

void traiterTrame(const uint8_t* t) {
  afficherTrame(t);
  afficherTrameLcd(t);

  uint8_t dest = lireDest(t);
  if (dest == ID_RELAIS) {
    // Trame adressee au relais lui-meme : rien de prevu dans le protocole pour l'instant.
    nbIgnorees++;
    DEBUGLN(F("   -> adressee au relais lui-meme : non geree"));
    return;
  }

  int8_t cible = cibleRelais(dest, lireExp(t));
  if (cible < 0) {
    nbIgnorees++;
    DEBUGLN(F("   -> ignoree : ce couple expediteur/destinataire ne passe pas par le relais"));
    return;
  }

  if (envoyerVers(cible, t)) {
    nbRelayees++;
    DEBUG(F("   -> relayee vers "));
    afficherNoeud(cible);
#if MODE_UNICAST
    DEBUG(F(", DL "));
    afficherAdresse(dlCourant);
#else
    DEBUG(F(", en broadcast"));
#endif
    DEBUGLN();
    digitalWrite(LED_BUILTIN, HIGH);
    ledAllumeeMs = millis();
  } else {
    DEBUGLN(F("   ! echec du changement de destination radio (ATDL)"));
  }
}

// ============================ SETUP / LOOP =============================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
#if XBEE_SUR_SOFTSERIAL
  Serial.begin(BAUD_DEBUG);
#endif
  xbee.begin(BAUD_XBEE);

  lcd.init();
  lcd.backlight();
  lcd.print(F("Relais 100"));
  lcd.setCursor(0, 1);
  lcd.print(F("En attente..."));

  DEBUGLN(F("=== RELAIS (ID 100) ==="));
  DEBUG(F("Configuration XBee... "));
  if (configurerXBee()) {
    DEBUGLN(F("OK"));
  } else {
    DEBUGLN(F("ECHEC (verifier cablage, cavaliers du shield et baudrate)"));
    for (uint8_t i = 0; i < 10; i++) {  // clignotement rapide = erreur
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
  }
  dernierBilanMs = millis();
}

void loop() {
  if (debug) testActionneur();

  uint8_t trame[4];
  if (lireTrame(trame)) traiterTrame(trame);

  if (ledAllumeeMs && millis() - ledAllumeeMs > DUREE_LED_MS) {
    digitalWrite(LED_BUILTIN, LOW);
    ledAllumeeMs = 0;
  }

#if XBEE_SUR_SOFTSERIAL
  if (millis() - dernierBilanMs >= PERIODE_BILAN_MS) {
    dernierBilanMs = millis();
    DEBUG(F("[bilan] relayees=")); DEBUG(nbRelayees);
    DEBUG(F(" ignorees="));        DEBUG(nbIgnorees);
    DEBUG(F(" rejetees="));        DEBUGLN(nbRejetees);
  }
#endif
}
