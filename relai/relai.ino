/*
 * =====================================================================
 *  Projet Objets Communicants - ENSIM
 *  Noeud RELAIS (NODE_RELAI = 1) - Arthur & Lucas
 *  Materiel : Arduino Uno + shield XBee + XBee Pro (802.15.4)
 * =====================================================================
 *
 *  Role : faire passer les trames entre le Hub (0), hors de portee,
 *         et les noeuds qui ne l'atteignent pas directement :
 *         temperature interieure (4) et servomoteur (5).
 *         La liste est tenue par noeudDerriereRelais().
 *         IDs et adresses XBee : libraries/EcostoreNodes (ecostore_nodes.h).
 *
 *  Trame (4 octets), codee et decodee par libraries/FrameProtocol :
 *    [0] 0xAA                          synchro
 *    [1] dest(3) | exp(3) | cmd(2)     en-tete (cmd : READ, WRITE, ERROR)
 *    [2] valeur sur 8 bits
 *    [3] checksum = (octets 0 a 2) % 256
 *
 *  Routage :
 *    exp = Hub  et dest derriere le relais  ->  envoyee au noeud dest
 *    dest = Hub et exp derriere le relais   ->  envoyee au Hub
 *    tout le reste                          ->  ignoree
 *
 *  La trame est retransmise a l'identique (checksum inchange).
 * =====================================================================
 */

#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ecostore_nodes.h>
#include <frame_protocol.h>

// ============================ CONFIGURATION ============================

// true = le relais envoie en boucle les trames de test au servomoteur (testActionneur)
const bool debug = false;
const unsigned long DELAI_TEST_MS = 2000;

// Cablage du shield :
//   1 = XBee sur pins 2/3 (SoftwareSerial), moniteur serie USB dispo pour le debug
//   0 = XBee sur le Serial materiel (pins 0/1), pas de debug possible
#define XBEE_SUR_SOFTSERIAL 1
const uint8_t PIN_XBEE_RX = 2;   // relie a DOUT du XBee
const uint8_t PIN_XBEE_TX = 3;   // relie a DIN du XBee
const long    BAUD_DEBUG  = 115200;

// Mode d'envoi :
//   1 = unicast   : le relais change sa destination radio (ATDH/ATDL) avant chaque envoi
//   0 = broadcast : DH = 0, DL = 0xFFFF, tout le monde recoit, le tri se fait sur le champ dest
#define MODE_UNICAST 1

// Reseau (tableau : canal C, PAN ID 1234)
const char RADIO_CANAL[]  = "C";
const char RADIO_PAN_ID[] = "1234";

// Adresse 64 bits d'un module XBee (numero de serie SH/SL), reglee via ATDH/ATDL.
struct AdresseXBee {
  uint32_t dh;
  uint32_t dl;
};

// La lib ne fournit que des macros separees : on les range par NODE_* pour router.
const AdresseXBee ADRESSES_XBEE[] = {
  {XBEE_HUB_DH,             XBEE_HUB_DL},              // NODE_HUB
  {XBEE_RELAI_DH,           XBEE_RELAI_DL},            // NODE_RELAI
  {XBEE_TEMP_HUM_EXT_DH,    XBEE_TEMP_HUM_EXT_DL},     // NODE_TEMP_HUM_EXT
  {XBEE_PHOTORESISTANCE_DH, XBEE_PHOTORESISTANCE_DL},  // NODE_PHOTORESISTANCE
  {XBEE_TEMP_INT_DH,        XBEE_TEMP_INT_DL},         // NODE_TEMP_INT
  {XBEE_SERVO_STORE_DH,     XBEE_SERVO_STORE_DL},      // NODE_SERVO_STORE
};
const uint8_t NB_NOEUDS = sizeof(ADRESSES_XBEE) / sizeof(ADRESSES_XBEE[0]);

const AdresseXBee ADRESSE_BROADCAST = {0x00000000, 0x0000FFFF};

// Temporisations
const unsigned int  GT_USINE_MS      = 1000;  // guard time par defaut du XBee
const unsigned int  GT_RAPIDE_MS     = 50;    // guard time regle au demarrage (ATGT 32)
const unsigned long TIMEOUT_TRAME_MS = 100;   // une trame commencee doit finir avant ce delai
const unsigned long DUREE_LED_MS     = 60;
const unsigned long PERIODE_BILAN_MS = 10000;

// Ecran LCD 16x2 en I2C (SDA = A4, SCL = A5 sur Uno)
const uint8_t LCD_ADRESSE_I2C = 0x27;
const uint8_t LCD_COLONNES    = 16;
const uint8_t LCD_LIGNES      = 2;

// Noms d'au plus 6 caracteres pour tenir "exp > dest" sur une ligne de 16.
const char NOM_COURT[8][7] = {
  "Hub", "Relais", "TmpExt", "Photo", "TmpInt", "Servo", "?", "?"
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

uint8_t       fenetre[FRAME_TOTAL_SIZE];  // octets en cours de reception
uint8_t       nbOctets       = 0;
unsigned long dernierOctetMs = 0;

AdresseXBee   destCourante = {0, 0};  // destination radio actuellement reglee
bool          destConnue   = false;

unsigned long nbRelayees = 0, nbRejetees = 0, nbIgnorees = 0;
unsigned long ledAllumeeMs = 0, dernierBilanMs = 0;

// ============================== AFFICHAGE ==============================

void afficherOctet(uint8_t b) {
  if (b < 0x10) DEBUG('0');
  DEBUG(b, HEX);
}

// Nom lisible du noeud, pour les affichages en clair.
const __FlashStringHelper* nomNoeud(uint8_t id) {
  switch (id) {
    case NODE_HUB:             return F("Hub");
    case NODE_RELAI:           return F("Relais");
    case NODE_TEMP_HUM_EXT:    return F("Temp/humidite exterieure");
    case NODE_PHOTORESISTANCE: return F("Photoresistance");
    case NODE_TEMP_INT:        return F("Temperature interieure");
    case NODE_SERVO_STORE:     return F("Servomoteur");
    default:                   return F("inconnu");
  }
}

void afficherHex32(uint32_t valeur) {
  for (int8_t i = 28; i >= 0; i -= 4) DEBUG((uint8_t)((valeur >> i) & 0x0F), HEX);
}

// "0013A200 40A1B2C3"
// En mode transparent le XBee ne nous transmet pas l'adresse source du paquet,
// on ne peut donc afficher que l'adresse *attendue* d'apres le champ exp.
void afficherAdresse(const AdresseXBee& a) {
  afficherHex32(a.dh);
  DEBUG(' ');
  afficherHex32(a.dl);
}

// "Hub (0)" ou, avec adresse=true, "Hub (0), adresse attendue 0013A200 40A1B2C3"
void afficherNoeud(uint8_t id, bool adresse = false) {
  DEBUG(nomNoeud(id));
  DEBUG(F(" ("));
  DEBUG(id);
  DEBUG(')');
  if (adresse && id < NB_NOEUDS) {
    DEBUG(F(", adresse attendue "));
    afficherAdresse(ADRESSES_XBEE[id]);
  }
}

const __FlashStringHelper* nomCommande(FrameCmd_t cmd) {
  switch (cmd) {
    case FRAME_CMD_READ:  return F("READ");
    case FRAME_CMD_WRITE: return F("WRITE");
    case FRAME_CMD_ERROR: return F("ERROR");
    default:              return F("?");
  }
}

// Les 4 octets en hexa : "AA 4A 1F 73"
void afficherOctets(const uint8_t* t) {
  for (uint8_t i = 0; i < FRAME_TOTAL_SIZE; i++) {
    if (i) DEBUG(' ');
    afficherOctet(t[i]);
  }
}

void afficherMessage(const FrameMsg_t& msg) {
  uint8_t octets[FRAME_TOTAL_SIZE];
  frame_pack(&msg, octets);
  afficherOctets(octets);
}

// Trame decodee en texte, sur plusieurs lignes :
//   RX trame : AA 81 1D 48
//      de       : Hub (0), adresse attendue 0013A200 40A1B2C3
//      vers     : Temperature interieure (4)
//      commande : WRITE
//      valeur   : 29 (0x1D)
void afficherTrame(const FrameMsg_t& msg) {
  DEBUG(F("RX trame : "));
  afficherMessage(msg);
  DEBUGLN();

  DEBUG(F("   de       : "));  afficherNoeud(msg.src_id, true);  DEBUGLN();
  DEBUG(F("   vers     : "));  afficherNoeud(msg.dest_id);       DEBUGLN();
  DEBUG(F("   commande : "));  DEBUGLN(nomCommande(msg.cmd));

  DEBUG(F("   valeur   : "));  DEBUG(msg.value);
  DEBUG(F(" (0x"));            DEBUG(msg.value, HEX);
  DEBUGLN(')');
}

// Ligne 1 : "TmpInt > Hub"   Ligne 2 : "WRITE val 24"
void afficherTrameLcd(const FrameMsg_t& msg) {
  lcd.clear();
  lcd.print(NOM_COURT[msg.src_id]);
  lcd.print(F(" > "));
  lcd.print(NOM_COURT[msg.dest_id]);
  lcd.setCursor(0, 1);
  lcd.print(nomCommande(msg.cmd));
  lcd.print(F(" val "));
  lcd.print(msg.value);
}

// ============================== RECEPTION ==============================

// Apres un rejet, le 0xAA de depart etait peut-etre une valeur :
// on cherche une autre synchro dans les octets deja recus au lieu de tout jeter.
// (frame_parse_byte de la lib n'a ni cette resynchro ni de timeout, d'ou cette lecture maison.)
void resynchroniser() {
  uint8_t i = 1;
  while (i < FRAME_TOTAL_SIZE && fenetre[i] != FRAME_START_BYTE) i++;
  nbOctets = FRAME_TOTAL_SIZE - i;
  memmove(fenetre, fenetre + i, nbOctets);
}

void signalerRejet() {
  nbRejetees++;
  DEBUG(F("! trame rejetee : "));
  afficherOctets(fenetre);
  // 0b10 est la seule valeur de cmd que frame_unpack refuse
  DEBUGLN((fenetre[1] & 0x03) == 0x02 ? F(" (commande 10 reservee)") : F(" (checksum faux)"));
}

// Renvoie true et remplit 'msg' quand une trame valide est recue.
bool lireTrame(FrameMsg_t& msg) {
  if (nbOctets > 0 && millis() - dernierOctetMs > TIMEOUT_TRAME_MS) {
    nbOctets = 0;  // trame incomplete abandonnee
  }

  while (xbee.available()) {
    uint8_t b = xbee.read();
    dernierOctetMs = millis();

    if (nbOctets == 0 && b != FRAME_START_BYTE) continue;  // on attend la synchro
    fenetre[nbOctets++] = b;
    if (nbOctets < FRAME_TOTAL_SIZE) continue;

    if (frame_unpack(fenetre, &msg)) {
      nbOctets = 0;
      return true;
    }
    signalerRejet();
    resynchroniser();
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

bool envoyerATHex(const __FlashStringHelper* cmd, uint32_t valeur) {
  char hex[9];
  sprintf(hex, "%lX", (unsigned long)valeur);
  return envoyerAT(cmd, hex);
}

bool quitterModeCommande() {
  xbee.print(F("ATCN\r"));
  return attendreOK(500);
}

bool adresseRenseignee(uint8_t id) {
  return ADRESSES_XBEE[id].dl != 0;
}

// Tant qu'une adresse est encore a 0 dans ecostore_nodes.h (TODO), on envoie en broadcast.
const AdresseXBee& adresseVers(uint8_t id) {
#if MODE_UNICAST
  if (adresseRenseignee(id)) return ADRESSES_XBEE[id];
#else
  (void)id;
#endif
  return ADRESSE_BROADCAST;
}

// Configure le XBee du relais a chaque demarrage (pas de ATWR : rien n'est ecrit en flash).
bool configurerXBee() {
  if (!entrerModeCommande(GT_USINE_MS)) return false;

  const AdresseXBee& dest = adresseVers(NODE_HUB);

  bool ok = true;
  ok &= envoyerAT(F("ATAP"), "0");           // mode transparent
  ok &= envoyerAT(F("ATCH"), RADIO_CANAL);
  ok &= envoyerAT(F("ATID"), RADIO_PAN_ID);
  ok &= envoyerATHex(F("ATDH"), dest.dh);    // DH != 0 : adressage 64 bits, MY ignore
  ok &= envoyerATHex(F("ATDL"), dest.dl);
  ok &= envoyerAT(F("ATGT"), "32");          // guard time 0x32 = 50 ms
  ok &= quitterModeCommande();

  if (ok) { destCourante = dest; destConnue = true; }
  return ok;
}

void signalerAdressesManquantes() {
  for (uint8_t id = 0; id < NB_NOEUDS; id++) {
    if (adresseRenseignee(id)) continue;
    DEBUG(F("! adresse XBee non renseignee pour "));
    afficherNoeud(id);
    DEBUGLN(F(" : envoi en broadcast"));
  }
}

// Change la destination radio, seulement si elle est differente de l'actuelle.
// DH etant le meme prefixe Digi pour tous les modules, en general seul ATDL part.
bool changerDestinationRadio(const AdresseXBee& adresse) {
  bool memeDh = destConnue && adresse.dh == destCourante.dh;
  if (memeDh && adresse.dl == destCourante.dl) return true;
  if (!entrerModeCommande(GT_RAPIDE_MS)) return false;

  bool ok = memeDh || envoyerATHex(F("ATDH"), adresse.dh);
  ok = ok && envoyerATHex(F("ATDL"), adresse.dl);
  ok = quitterModeCommande() && ok;

  if (ok) { destCourante = adresse; destConnue = true; }
  else    { destConnue = false; }
  return ok;
}

// =============================== ROUTAGE ===============================

// Noeuds hors de portee du Hub : c'est pour eux que le relais existe.
// Ajouter ou retirer un noeud ici suffit, les deux sens suivent.
bool noeudDerriereRelais(uint8_t id) {
  return id == NODE_TEMP_INT       // P20
      || id == NODE_SERVO_STORE;   // P20
}

// Renvoie l'ID vers lequel relayer la trame, ou -1 pour l'ignorer.
int8_t cibleRelais(uint8_t dest, uint8_t exp) {
  if (exp == NODE_HUB && noeudDerriereRelais(dest)) return dest;      // Hub -> noeud
  if (dest == NODE_HUB && noeudDerriereRelais(exp)) return NODE_HUB;  // noeud -> Hub
  return -1;
}

bool envoyerVers(uint8_t id, const FrameMsg_t& msg) {
  if (!changerDestinationRadio(adresseVers(id))) return false;
  uint8_t trame[FRAME_TOTAL_SIZE];
  frame_pack(&msg, trame);
  xbee.write(trame, FRAME_TOTAL_SIZE);
  return true;
}

// Commandes a confirmer avec le groupe servo.
const FrameMsg_t TRAMES_TEST_SERVO[] = {
  {NODE_SERVO_STORE, NODE_HUB, FRAME_CMD_WRITE, 1},  // ouvrir
  {NODE_SERVO_STORE, NODE_HUB, FRAME_CMD_WRITE, 0},  // fermer
  {NODE_SERVO_STORE, NODE_HUB, FRAME_CMD_READ,  0},  // demande d'etat
};

void testActionneur() {
  for (const FrameMsg_t& msg : TRAMES_TEST_SERVO) {
    DEBUG(F("TX test : "));
    afficherMessage(msg);
    DEBUGLN(envoyerVers(NODE_SERVO_STORE, msg) ? F("") : F("  ! echec ATDH/ATDL"));
    afficherTrameLcd(msg);
    delay(DELAI_TEST_MS);
  }
}

void traiterTrame(const FrameMsg_t& msg) {
  afficherTrame(msg);
  afficherTrameLcd(msg);

  if (msg.dest_id == NODE_RELAI) {
    // Trame adressee au relais lui-meme : rien de prevu dans le protocole pour l'instant.
    nbIgnorees++;
    DEBUGLN(F("   -> adressee au relais lui-meme : non geree"));
    return;
  }

  int8_t cible = cibleRelais(msg.dest_id, msg.src_id);
  if (cible < 0) {
    nbIgnorees++;
    DEBUGLN(F("   -> ignoree : ce couple expediteur/destinataire ne passe pas par le relais"));
    return;
  }

  if (envoyerVers(cible, msg)) {
    nbRelayees++;
    DEBUG(F("   -> relayee vers "));
    afficherNoeud(cible);
    if (destCourante.dl == ADRESSE_BROADCAST.dl) {
      DEBUG(F(", en broadcast"));
    } else {
      DEBUG(F(", adresse "));
      afficherAdresse(destCourante);
    }
    DEBUGLN();
    digitalWrite(LED_BUILTIN, HIGH);
    ledAllumeeMs = millis();
  } else {
    DEBUGLN(F("   ! echec du changement de destination radio (ATDH/ATDL)"));
  }
}

// ============================ SETUP / LOOP =============================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
#if XBEE_SUR_SOFTSERIAL
  Serial.begin(BAUD_DEBUG);
#endif
  xbee.begin(XBEE_BAUD);

  lcd.init();
  lcd.backlight();
  lcd.print(F("Relais 1"));
  lcd.setCursor(0, 1);
  lcd.print(F("En attente..."));

  DEBUGLN(F("=== RELAIS (ID 1) ==="));
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
#if MODE_UNICAST
  signalerAdressesManquantes();
#endif
  dernierBilanMs = millis();
}

void loop() {
  if (debug) testActionneur();

  FrameMsg_t msg;
  if (lireTrame(msg)) traiterTrame(msg);

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
