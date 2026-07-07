#include <Arduino.h>
/* =====================================================================
 *  Chariot motorisé sur mât vertical — firmware v2 « boucle fermée »
 *
 *  Cible   : Arduino Uno/Nano (ATmega328P), driver pas-à-pas STEP/DIR
 *  Moteur  : StepperOnline 23E1KBK20-20 (NEMA23 closed-loop, frein
 *            électromagnétique 24V, codeur incrémental optique 1000 PPR)
 *
 *  Architecture :
 *   - loop() 100 % non bloquant : aucune attente > quelques µs.
 *     (seules exceptions : impulsion STEP ~10 µs et setup DIR 5 µs)
 *   - Génération de pas par échéancier micros() (machine à états).
 *   - Codeur lu par interruption INT0 (D2, canal A) + lecture directe
 *     du canal B (D6) dans l'ISR → décodage x2 = 2000 pts/tour.
 *   - Asservissement de supervision : comparaison continue entre la
 *     position théorique (pas émis) et la position mesurée (codeur).
 *     Écart > STALL_TOL_MM → arrêt + frein + ERR:STALL.
 *   - Fin de POS: → rattrapage fin par micro-pas lents (3 essais max)
 *     puis recalage de la position de référence sur le codeur.
 *   - Télémétrie 60 Hz : POS:...,CNT:...,MIN:.,MAX:.,BRAKE:.,HOMED:.
 *
 *  Signe codeur : auto-calibré pendant le homing (phase montée). Si le
 *  codeur ne compte pas alors que le moteur bouge → WARN:ENC_ABSENT et
 *  la supervision est désactivée (fonctionnement dégradé boucle ouverte,
 *  explicitement signalé — jamais silencieux).
 * ===================================================================== */

// ── Broches ──────────────────────────────────────────────────────────
#define PUL_PIN        4   // STEP vers driver (3)
#define DIR_PIN        5   // DIR vers driver (4)
#define ENA_PIN        6   // Enable driver (LOW à l'init, inchangé) (5)
#define ALRM_PIN       7   // ALARM driver (actif LOW, pull-up interne) (6)
#define ENC_A_PIN      2   // Codeur canal A (EA+) — INT0 (2)
#define ENC_B_PIN      3   // Codeur canal B (EB+) — lecture directe (6)
#define ENDSTOP_MAX   10   // Fin de course HAUT, actif LOW, pull-up interne (10)
#define ENDSTOP_MIN    9   // Fin de course BAS,  actif LOW, pull-up interne (9)
#define BRAKE_RELAY   11   // Relais frein 24V : HIGH = libéré, LOW = serré (11)



// ── Paramètres machine ───────────────────────────────────────────────
const long  STEPS_PER_REV     = 3200;                       // driver en 1/16
const float MM_PER_REV        = 60.0;                       // pignon 60 mm/tour
const float STEPS_PER_MM      = STEPS_PER_REV / MM_PER_REV; // 53.33
const long  ENC_COUNTS_PER_REV = 2000;                      // 1000 PPR décodés x2
const float ENC_COUNTS_PER_MM = ENC_COUNTS_PER_REV / MM_PER_REV; // 33.33

// Périodes de pas en µs (période COMPLÈTE par pas ; l'ancien code
// utilisait des demi-périodes : 500 µs demi-période ≡ 1000 µs ici).
const unsigned int PERIOD_MOVE_SLOW      = 2000; // départ/arrivée très doux
const unsigned int PERIOD_MOVE_FAST_UP   = 300; // croisière montée
const unsigned int PERIOD_MOVE_FAST_DOWN = 300; // croisière descente.
// Volontairement identiques par défaut (l'ancien commentaire « vitesses
// différenciées » ne correspondait à rien). La descente étant aidée par
// la gravité et la montée non, on peut ralentir la montée (augmenter
// PERIOD_MOVE_FAST_UP) si des pertes de pas apparaissent — mais la
// supervision codeur les détecte désormais, donc on garde la symétrie
// tant que le terrain ne prouve pas le contraire.
const unsigned int PERIOD_HOMING_SEEK    = 300;  // recherche FDC (ex 150 µs demi)
const unsigned int PERIOD_HOMING_BACKOFF = 400;  // recul 3 mm (ex 200 µs demi)
const unsigned int PERIOD_HOMING_FINAL   = 200;  // recul final (ex 100 µs demi)
const unsigned int PERIOD_JOG            = 400;  // jog (ex 200 µs demi)
const unsigned int STEP_PULSE_US         = 10;   // largeur impulsion STEP
const unsigned int DIR_SETUP_US          = 5;    // setup DIR avant STEP

// ── Mode vitesse VEL:<mm/s> (téléop joystick) ────────────────────────
// Vitesse continue signée : VEL:>0 monte, VEL:<0 descend, VEL:0 arrête.
// La période de pas est calculée depuis la consigne : period_µs =
// 1e6 / (|v| * STEPS_PER_MM). Un WATCHDOG homme-mort arrête le chariot
// (+ frein) si aucun VEL n'est reçu pendant VEL_TIMEOUT_MS : le PC doit
// donc RÉÉMETTRE la consigne périodiquement (≥ ~10 Hz).
const float        VEL_MAX_MM_S   = 100.0; // vitesse max autorisée (clamp)
const float        VEL_MIN_MM_S   = 1.0;   // sous ce seuil -> arrêt
const unsigned int VEL_PERIOD_MIN = 187;   // ≈ 1e6/(100*53.33) : plancher période
const unsigned int VEL_PERIOD_MAX = 18750; // ≈ 1e6/(1*53.33)   : plafond période
const unsigned int VEL_TIMEOUT_MS = 300;   // watchdog homme-mort (sans VEL -> stop+frein)

// ── Supervision boucle fermée ────────────────────────────────────────
#define ENCODER_SUPERVISION 1        // 0 = boucle ouverte pure (banc de test)
const float STALL_TOL_MM        = 5.0;  // écart pas/codeur → décrochage
const float STALL_MIN_TRAVEL_MM = 3.0;  // grâce au démarrage (relais frein…)
const float POS_TOL_MM          = 0.3;  // tolérance d'arrivée POS:
const uint8_t CORRECT_MAX       = 3;    // essais de rattrapage fin max
const unsigned int SUPERVISION_PERIOD_MS = 50;
// Vérification codeur pendant le homing (phase montée) :
const long  ENC_CHECK_STEPS     = 800;  // après 15 mm de montée...
const long  ENC_DEAD_COUNTS     = 50;   // ...< 50 counts (~500 attendus) = absent

// ── Divers ───────────────────────────────────────────────────────────
const unsigned int  DELAI_RELAIS_MS      = 10;    // stabilisation relais frein
const unsigned long TELEMETRY_PERIOD_US  = 16667; // 60 Hz
const unsigned int  DEBOUNCE_ASSERT_MS   = 2;     // anti-parasite FDC (déclenchement)
const unsigned int  DEBOUNCE_RELEASE_MS  = 20;    // anti-rebond FDC (relâchement)
const float HOMING_MAX_TRAVEL_MM = 700.0 * 1.2;   // garde-fou course homing
const float HOMING_BACKOFF_MM    = 3.0;
const long  HOMING_MAX_STEPS   = (long)(HOMING_MAX_TRAVEL_MM * STEPS_PER_MM);
const long  BACKOFF_STEPS      = (long)(HOMING_BACKOFF_MM * STEPS_PER_MM);
const unsigned long FDC_TEST_MS = 10000;

// ── Sens ─────────────────────────────────────────────────────────────
const bool UP   = false;   // DIR LOW  (identique v1)
const bool DOWN = true;    // DIR HIGH

// ── Codeur (partagé avec l'ISR) ──────────────────────────────────────
volatile long encCount = 0;
int8_t encSign     = 1;      // auto-calibré au homing
bool   encVerified = false;  // codeur vu vivant + signe validé
bool   supEnabled  = (ENCODER_SUPERVISION != 0);

// ── État machine ─────────────────────────────────────────────────────
enum State : uint8_t { ST_IDLE, ST_PRE, ST_MOVE, ST_CORRECT, ST_JOG,
                       ST_VEL, ST_HOMING, ST_POST, ST_FDC };
enum Pending : uint8_t { PEND_NONE, PEND_MOVE, PEND_JOG, PEND_VEL, PEND_HOMING };
enum HomingPhase : uint8_t { HP_SEEK_MIN, HP_BACKOFF1, HP_SEEK_MAX,
                             HP_RETURN_MIN, HP_BACKOFF2 };

State   state = ST_IDLE;
Pending pending = PEND_NONE;
float   pendTarget = 0.0;
bool    pendJogDir = UP;
unsigned long preStart = 0;    // entrée dans ST_PRE / ST_POST (millis)
unsigned long postStart = 0;

bool  isHomed      = false;
float courseMm     = 0.0;
long  stepCount    = 0;        // position théorique en pas (0 = home)
bool  brakeReleased = false;
bool  manualBrake  = false;    // frein libéré manuellement (BRAKE:1 à l'arrêt)

// Mouvement POS: / correction
struct MoveState {
  bool dir;
  long total, done, ramp;
  unsigned int fast;           // période de croisière
  float target;
  uint8_t retries;
} mv;
unsigned long nextStepDue = 0; // micros()

// Jog
bool jogDir = UP;

// Mode vitesse VEL:
bool          velDir = UP;
unsigned int  velPeriod = PERIOD_JOG;   // période de pas courante (µs)
bool          pendVelDir = UP;
unsigned int  pendVelPeriod = PERIOD_JOG;
unsigned long velLastCmdMs = 0;         // dernier VEL reçu (watchdog)

// Homing
HomingPhase hp;
long homSafety = 0, homStepsToMax = 0, homBackoffLeft = 0;
long homEncPhaseStart = 0, homEncDelta = 0;
bool homSignChecked = false;

// Supervision décrochage
float supStepRef = 0, supEncRef = 0;
unsigned long supLastCheck = 0;

// Télémétrie
unsigned long nextTelemetry = 0;

// Test FDC
unsigned long fdcStart = 0;
int  fdcCntMin = 0, fdcCntMax = 0;
bool fdcLastMin = false, fdcLastMax = false, fdcInitMin = false, fdcInitMax = false;

// Réception série
char    rxBuf[24];
uint8_t rxLen = 0;
bool    rxDiscard = false;

// ── Fins de course : anti-rebond asymétrique non bloquant ───────────
// Déclenchement : 2 ms de LOW stable (filtre les parasites µs induits
// par le câble moteur, comme le faisait la relecture de la v1, sans
// bloquer ; surcourse max ≈ 0,13 mm à la vitesse de homing).
// Relâchement : 20 ms de HIGH stable (rebonds mécaniques).
struct Endstop {
  uint8_t pin;
  bool active;
  bool pendingChange;
  unsigned long tChange;
};
Endstop esMin = {ENDSTOP_MIN, false, false, 0};
Endstop esMax = {ENDSTOP_MAX, false, false, 0};

void updateEndstop(Endstop &e) {
  bool raw = (digitalRead(e.pin) == LOW);
  if (raw == e.active) {
    e.pendingChange = false;
    return;
  }
  if (!e.pendingChange) {
    e.pendingChange = true;
    e.tChange = millis();
    return;
  }
  unsigned int need = raw ? DEBOUNCE_ASSERT_MS : DEBOUNCE_RELEASE_MS;
  if (millis() - e.tChange >= need) {
    e.active = raw;
    e.pendingChange = false;
  }
}

// ── ISR codeur : décodage x2 sur les fronts du canal A ──────────────
// D2 = PD2, D6 = PD6 → lecture directe du port (rapide, ~1 µs).
void isrEncoderA() {
  uint8_t pins = PIND;
  bool a = pins & _BV(PD2);
  bool b = pins & _BV(PD6);
  if (a == b) encCount++;
  else        encCount--;
}

long encRead() {
  noInterrupts();
  long c = encCount;
  interrupts();
  return c;
}

float encoderMm() { return (encRead() * (long)encSign) / ENC_COUNTS_PER_MM; }
float stepMm()    { return stepCount / STEPS_PER_MM; }

// Position de référence : codeur si vérifié, sinon comptage de pas.
float positionMm() {
  return (supEnabled && encVerified) ? encoderMm() : stepMm();
}

void resyncOnEncoder() {
  if (supEnabled && encVerified) stepCount = lround(encoderMm() * STEPS_PER_MM);
}

// ── Frein / moteur ───────────────────────────────────────────────────
void setBrake(bool release) {
  digitalWrite(BRAKE_RELAY, release ? HIGH : LOW);
  brakeReleased = release;
}

void setDir(bool dir) {
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
}

void doPulse(bool dir) {
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PUL_PIN, LOW);
  stepCount += (dir == UP) ? 1 : -1;
}

// ── Supervision décrochage ───────────────────────────────────────────
void supReset() {
  supStepRef = stepMm();
  supEncRef  = encoderMm();
  supLastCheck = millis();
}

void printPos() {
  Serial.print(F("POS:"));
  Serial.print(positionMm(), 1);
}

void superviseStall() {
  if (!supEnabled || !encVerified) return;
  if (state != ST_MOVE && state != ST_CORRECT && state != ST_JOG &&
      state != ST_VEL && state != ST_HOMING) return;
  if (millis() - supLastCheck < SUPERVISION_PERIOD_MS) return;
  supLastCheck = millis();

  float stepD = stepMm() - supStepRef;
  float encD  = encoderMm() - supEncRef;
  if (fabs(stepD) < STALL_MIN_TRAVEL_MM) return;  // grâce au démarrage

  float dev = fabs(stepD - encD);
  if (dev <= STALL_TOL_MM) return;

  // Décrochage / obstruction : arrêt immédiat + frein.
  bool encSilent = fabs(encD) < 1.0;   // codeur muet alors que pas émis
  state = ST_IDLE;
  setBrake(false);
  if (encSilent) {
    // Impossible de distinguer « moteur bloqué net » de « codeur mort » :
    // dans les deux cas on stoppe, mais la position n'est plus fiable.
    encVerified = false;
    isHomed = false;
    Serial.print(F("ERR:ENC_OR_STALL,"));
  } else {
    resyncOnEncoder();                 // le codeur reste la vérité
    Serial.print(F("ERR:STALL,"));
  }
  printPos();
  Serial.print(F(",DEV:"));
  Serial.println(dev, 1);
}

// ── Séquencement frein → mouvement ───────────────────────────────────
void startPre(Pending p) {
  manualBrake = false;
  pending = p;
  setBrake(true);
  preStart = millis();
  state = ST_PRE;
}

void enterPost() {
  postStart = millis();
  state = ST_POST;
}

void emergencyStop(const __FlashStringHelper *why) {
  state = ST_IDLE;
  pending = PEND_NONE;
  manualBrake = false;
  setBrake(false);                     // frein immédiat, pas d'attente
  resyncOnEncoder();
  Serial.print(F("MSG:STOPPED,"));
  Serial.print(why);
  Serial.print(F(","));
  printPos();
  Serial.println();
}

// ── Planification d'un mouvement POS: ───────────────────────────────
// Appelé frein déjà libéré (depuis ST_PRE ou retarget en vol).
void planMove(float target) {
  target = constrain(target, 0.0, courseMm);
  float delta = target - positionMm();
  mv.target = target;
  mv.retries = 0;

  if (fabs(delta) < POS_TOL_MM) {
    Serial.println(F("MSG:DEJA_EN_POSITION"));
    enterPost();
    return;
  }

  mv.dir   = (delta > 0) ? UP : DOWN;
  mv.total = lround(fabs(delta) * STEPS_PER_MM);
  mv.done  = 0;
  mv.ramp  = min(mv.total / 4, 500L);
  mv.fast  = (mv.dir == UP) ? PERIOD_MOVE_FAST_UP : PERIOD_MOVE_FAST_DOWN;

  Serial.print(mv.dir == UP ? F("MSG:MONTEE,") : F("MSG:DESCENTE,"));
  Serial.print(F("CIBLE:"));
  Serial.print(target, 1);
  Serial.print(F(",PAS:"));
  Serial.println(mv.total);

  setDir(mv.dir);
  supReset();
  nextStepDue = micros();
  state = ST_MOVE;
}

// Période du pas courant (profil trapézoïdal, même forme que la v1).
unsigned int moveInterval() {
  long i = mv.done;
  long span = (long)PERIOD_MOVE_SLOW - (long)mv.fast;
  if (mv.ramp > 0) {
    if (i < mv.ramp)
      return PERIOD_MOVE_SLOW - (unsigned int)(span * i / mv.ramp);
    if (i >= mv.total - mv.ramp)
      return mv.fast + (unsigned int)(span * (i - (mv.total - mv.ramp)) / mv.ramp);
  }
  return mv.fast;
}

void scheduleNext(unsigned int interval) {
  nextStepDue += interval;
  // Anti-rafale : si on a pris du retard (télémétrie…), on repart de
  // maintenant plutôt que d'émettre des pas en rafale pour rattraper.
  if ((long)(micros() - nextStepDue) > (long)interval)
    nextStepDue = micros() + interval;
}

// ── Fin de mouvement : rattrapage fin puis recalage codeur ──────────
void finishMove() {
  resyncOnEncoder();
  Serial.print(F("MSG:ARRIVE,"));
  printPos();
  Serial.print(F(",CIBLE:"));
  Serial.print(mv.target, 1);
  if (courseMm > 0) {
    Serial.print(F(",PCT:"));
    Serial.print(positionMm() / courseMm * 100.0, 1);
  }
  if (mv.retries > 0) {
    Serial.print(F(",CORR:"));
    Serial.print(mv.retries);
  }
  Serial.println();
  enterPost();
}

void beginCorrectionOrFinish() {
  if (supEnabled && encVerified) {
    float err = mv.target - encoderMm();
    if (fabs(err) > POS_TOL_MM && fabs(err) < STALL_TOL_MM &&
        mv.retries < CORRECT_MAX) {
      mv.retries++;
      mv.dir   = (err > 0) ? UP : DOWN;
      mv.total = lround(fabs(err) * STEPS_PER_MM);
      mv.done  = 0;
      mv.ramp  = 0;
      mv.fast  = PERIOD_MOVE_SLOW;    // rattrapage lent
      if (mv.total > 0) {
        setDir(mv.dir);
        supReset();
        nextStepDue = micros();
        state = ST_CORRECT;
        return;
      }
    }
    if (fabs(err) > POS_TOL_MM) {
      Serial.print(F("WARN:POS_TOL,ERR_MM:"));
      Serial.println(err, 2);
    }
  }
  finishMove();
}

// FDC touché pendant un mouvement / jog.
void fdcAbort(bool isMax) {
  Serial.print(isMax ? F("WARN:FDC_MAX,") : F("WARN:FDC_MIN,"));
  if (supEnabled && encVerified) {
    resyncOnEncoder();                // position vraie conservée
  } else if (isHomed) {
    // Boucle ouverte : on recale approximativement comme la v1.
    stepCount = isMax ? lround(courseMm * STEPS_PER_MM) : 0;
  }
  printPos();
  Serial.println();
  enterPost();                        // frein engagé après stabilisation
}

// ── Homing (machine à états, non bloquant) ───────────────────────────
void homingFail(const __FlashStringHelper *msg) {
  Serial.print(F("ERR:HOMING,"));
  Serial.println(msg);
  isHomed = false;
  enterPost();                        // frein ! (la v1 le laissait libéré)
}

void homingStart() {
  isHomed  = false;
  courseMm = 0.0;
  hp = HP_SEEK_MIN;
  homSafety = 0;
  homSignChecked = false;
  supReset();
  setDir(DOWN);
  nextStepDue = micros();
  Serial.println(F("MSG:HOMING,1/4,SEEK_MIN"));
}

void homingDone() {
  float courseSteps = (float)(homStepsToMax - BACKOFF_STEPS) / STEPS_PER_MM;
  if (supEnabled && encVerified) {
    float courseEnc = fabs((float)homEncDelta) / ENC_COUNTS_PER_MM
                      - HOMING_BACKOFF_MM;
    if (fabs(courseEnc - courseSteps) > STALL_TOL_MM) {
      Serial.print(F("WARN:HOMING_DEVIATION,PAS:"));
      Serial.print(courseSteps, 1);
      Serial.print(F(",ENC:"));
      Serial.println(courseEnc, 1);
    }
    courseMm = courseEnc;             // le codeur est la source de vérité
  } else {
    courseMm = courseSteps;
  }

  // Zéro machine : position actuelle (3 mm au-dessus du FDC MIN).
  noInterrupts();
  encCount = 0;
  interrupts();
  stepCount = 0;
  isHomed = true;
  Serial.print(F("MSG:HOMING_OK,COURSE:"));
  Serial.println(courseMm, 1);
  enterPost();
}

void homingTick() {
  unsigned int period;
  switch (hp) {
    case HP_SEEK_MIN:
    case HP_RETURN_MIN: period = PERIOD_HOMING_SEEK;    break;
    case HP_BACKOFF1:   period = PERIOD_HOMING_BACKOFF; break;
    case HP_SEEK_MAX:   period = PERIOD_HOMING_SEEK;    break;
    default:            period = PERIOD_HOMING_FINAL;   break;
  }
  if ((long)(micros() - nextStepDue) < 0) return;

  switch (hp) {
    case HP_SEEK_MIN:
      if (esMin.active) {
        Serial.println(F("MSG:HOMING,2/4,BACKOFF"));
        hp = HP_BACKOFF1;
        homBackoffLeft = BACKOFF_STEPS;
        setDir(UP);
        supReset();
        return;
      }
      doPulse(DOWN);
      if (++homSafety > HOMING_MAX_STEPS) { homingFail(F("MIN_INTROUVABLE")); return; }
      break;

    case HP_BACKOFF1:
      doPulse(UP);
      if (--homBackoffLeft <= 0) {
        Serial.println(F("MSG:HOMING,3/4,SEEK_MAX"));
        hp = HP_SEEK_MAX;
        homSafety = 0;
        homStepsToMax = 0;
        homEncPhaseStart = encRead();
        supReset();
      }
      break;

    case HP_SEEK_MAX:
      if (esMax.active) {
        homEncDelta = encRead() - homEncPhaseStart;
        Serial.println(F("MSG:HOMING,4/4,RETURN_MIN"));
        hp = HP_RETURN_MIN;
        homSafety = 0;
        setDir(DOWN);
        supReset();
        return;
      }
      doPulse(UP);
      homStepsToMax++;

      // Vérification vie + signe du codeur après 15 mm de montée.
      if (!homSignChecked && homStepsToMax >= ENC_CHECK_STEPS) {
        homSignChecked = true;
        long delta = encRead() - homEncPhaseStart;
        if (supEnabled && labs(delta) < ENC_DEAD_COUNTS) {
          encVerified = false;
          Serial.println(F("WARN:ENC_ABSENT,SUPERVISION_OFF"));
        } else if (supEnabled) {
          if ((long)encSign * delta < 0) {
            encSign = -encSign;
            Serial.println(F("MSG:ENC_SIGN_INVERSE"));
          }
          encVerified = true;
          supReset();                 // nouvelle référence, signe correct
        }
      }
      if (++homSafety > HOMING_MAX_STEPS) { homingFail(F("MAX_INTROUVABLE")); return; }
      break;

    case HP_RETURN_MIN:
      if (esMin.active) {
        hp = HP_BACKOFF2;
        homBackoffLeft = BACKOFF_STEPS;
        setDir(UP);
        supReset();
        return;
      }
      doPulse(DOWN);
      if (++homSafety > HOMING_MAX_STEPS) { homingFail(F("RETOUR_MIN_IMPOSSIBLE")); return; }
      break;

    case HP_BACKOFF2:
      doPulse(UP);
      if (--homBackoffLeft <= 0) { homingDone(); return; }
      break;
  }
  scheduleNext(period);
}

// ── Test FDC non bloquant (10 s, la télémétrie continue) ─────────────
void fdcTestStart() {
  fdcStart = millis();
  fdcCntMin = fdcCntMax = 0;
  fdcInitMin = fdcLastMin = esMin.active;
  fdcInitMax = fdcLastMax = esMax.active;
  Serial.print(F("MSG:FDC_TEST,10s,MIN(D8):"));
  Serial.print(fdcInitMin ? F("ACTIF") : F("ok"));
  Serial.print(F(",MAX(D7):"));
  Serial.println(fdcInitMax ? F("ACTIF") : F("ok"));
  state = ST_FDC;
}

void fdcTestTick() {
  float t = (millis() - fdcStart) / 1000.0;
  if (esMin.active != fdcLastMin) {
    fdcLastMin = esMin.active;
    if (fdcLastMin) fdcCntMin++;
    Serial.print(F("MSG:FDC_MIN_"));
    Serial.print(fdcLastMin ? F("ON,") : F("OFF,"));
    Serial.println(t, 1);
  }
  if (esMax.active != fdcLastMax) {
    fdcLastMax = esMax.active;
    if (fdcLastMax) fdcCntMax++;
    Serial.print(F("MSG:FDC_MAX_"));
    Serial.print(fdcLastMax ? F("ON,") : F("OFF,"));
    Serial.println(t, 1);
  }
  if (millis() - fdcStart >= FDC_TEST_MS) {
    Serial.print(F("MSG:FDC_BILAN,MIN:"));
    Serial.print(fdcCntMin);
    Serial.print(fdcInitMin ? F("(ERREUR_ACTIF_DEPART)") : F(""));
    Serial.print(F(",MAX:"));
    Serial.print(fdcCntMax);
    Serial.println(fdcInitMax ? F("(ERREUR_ACTIF_DEPART)") : F(""));
    state = ST_IDLE;
  }
}

// ── Commandes série ──────────────────────────────────────────────────
void cmdBrake(bool release) {
  bool moving = (state != ST_IDLE && state != ST_FDC);
  if (release) {
    if (moving) {
      Serial.println(F("MSG:BRAKE_DEJA_GERE"));   // libéré par le mouvement
    } else {
      manualBrake = true;
      setBrake(true);
      Serial.println(F("WARN:BRAKE_LIBERE_MANUEL,RISQUE_CHUTE"));
    }
  } else {
    if (moving) {
      emergencyStop(F("BRAKE_0"));                // serrer = stopper d'abord
    } else {
      manualBrake = false;
      setBrake(false);
      Serial.println(F("MSG:BRAKE_SERRE"));
    }
  }
}

void cmdPos(float target) {
  if (!isHomed) {
    Serial.println(F("MSG:REFUS,HOMING_REQUIS"));
    return;
  }
  if (state == ST_MOVE || state == ST_CORRECT) {  // nouvelle consigne en vol
    Serial.println(F("MSG:RETARGET"));
    planMove(target);
    return;
  }
  if (state == ST_PRE && pending == PEND_MOVE) {
    pendTarget = target;
    Serial.println(F("MSG:RETARGET"));
    return;
  }
  if (state != ST_IDLE) {
    Serial.println(F("MSG:REFUS,BUSY"));
    return;
  }
  pendTarget = target;
  startPre(PEND_MOVE);
}

void cmdJog(bool dir) {
  if (state == ST_JOG) {                          // inversion en vol
    jogDir = dir;
    setDir(dir);
    supReset();
    return;
  }
  if (state != ST_IDLE) {
    Serial.println(F("MSG:REFUS,BUSY"));
    return;
  }
  pendJogDir = dir;
  startPre(PEND_JOG);
}

void cmdJogStop() {
  if (state == ST_JOG) {
    Serial.print(F("MSG:JOG_STOP,"));
    resyncOnEncoder();
    printPos();
    Serial.println();
    enterPost();
  } else if (state == ST_PRE && pending == PEND_JOG) {
    pending = PEND_NONE;
    enterPost();
  }
  // Sinon : déjà à l'arrêt, frein déjà géré — rien à faire.
}

void cmdHoming() {
  if (state != ST_IDLE) {
    Serial.println(F("MSG:REFUS,BUSY,STOP_D_ABORD"));
    return;
  }
  startPre(PEND_HOMING);
}

// ── Mode vitesse VEL:<mm/s> ─────────────────────────────────────────
unsigned int velPeriodFromSpeed(float vabs) {
  float p = 1000000.0 / (vabs * STEPS_PER_MM);
  if (p < (float)VEL_PERIOD_MIN) p = VEL_PERIOD_MIN;
  if (p > (float)VEL_PERIOD_MAX) p = VEL_PERIOD_MAX;
  return (unsigned int)p;
}

void cmdVel(float v) {
  float vabs = fabs(v);

  // Consigne quasi nulle = demande d'arrêt (frein serré).
  if (vabs < VEL_MIN_MM_S) {
    if (state == ST_VEL) {
      Serial.print(F("MSG:VEL_STOP,"));
      resyncOnEncoder();
      printPos();
      Serial.println();
      enterPost();
    } else if (state == ST_PRE && pending == PEND_VEL) {
      pending = PEND_NONE;
      enterPost();
    }
    return;
  }

  if (vabs > VEL_MAX_MM_S) vabs = VEL_MAX_MM_S;        // clamp vitesse
  bool dir = (v > 0) ? UP : DOWN;                       // >0 = montée
  unsigned int period = velPeriodFromSpeed(vabs);
  velLastCmdMs = millis();                              // rafraîchit le watchdog

  if (state == ST_VEL) {                                // mise à jour en vol
    if (dir != velDir) { setDir(dir); supReset(); }
    velDir = dir;
    velPeriod = period;
    return;
  }
  if (state == ST_PRE && pending == PEND_VEL) {         // encore en libération frein
    pendVelDir = dir;
    pendVelPeriod = period;
    return;
  }
  if (state != ST_IDLE) {                               // POS/homing/FDC en cours
    Serial.println(F("MSG:REFUS,BUSY"));
    return;
  }
  // Ne pas démarrer dans une butée déjà active (évite l'oscillation).
  if ((dir == UP && esMax.active) || (dir == DOWN && esMin.active)) {
    Serial.println(F("MSG:REFUS,FDC"));
    return;
  }
  pendVelDir = dir;
  pendVelPeriod = period;
  startPre(PEND_VEL);
}

void handleCommand(const char *cmd) {
  Serial.print(F("ACK "));                        // format v1 conservé
  Serial.println(cmd);

  if      (!strcmp(cmd, "STOP"))        emergencyStop(F("CMD"));
  else if (!strncmp(cmd, "BRAKE:", 6)) {
    if ((cmd[6] == '0' || cmd[6] == '1') && cmd[7] == '\0')
      cmdBrake(cmd[6] == '1');
    else
      Serial.println(F("MSG:INCONNU,BRAKE_INVALIDE"));
  }
  else if (!strncmp(cmd, "POS:", 4)) {
    char *end;
    float v = strtod(cmd + 4, &end);
    // Nombre invalide -> refus explicite (atof aurait renvoyé 0.0 et
    // envoyé le chariot en bas du mât sur une trame corrompue !).
    if (end == cmd + 4 || *end != '\0')
      Serial.println(F("MSG:INCONNU,POS_INVALIDE"));
    else
      cmdPos(v);
  }
  else if (!strncmp(cmd, "VEL:", 4)) {
    char *end;
    float v = strtod(cmd + 4, &end);
    if (end == cmd + 4 || *end != '\0')
      Serial.println(F("MSG:INCONNU,VEL_INVALIDE"));
    else
      cmdVel(v);
  }
  else if (!strcmp(cmd, "H"))           cmdHoming();
  else if (!strcmp(cmd, "UP_START"))    cmdJog(UP);
  else if (!strcmp(cmd, "DOWN_START"))  cmdJog(DOWN);
  else if (!strcmp(cmd, "UP_STOP"))     cmdJogStop();
  else if (!strcmp(cmd, "DOWN_STOP"))   cmdJogStop();
  else if (!strcmp(cmd, "FDC")) {
    if (state == ST_IDLE) fdcTestStart();
    else Serial.println(F("MSG:REFUS,BUSY"));
  }
  else {
    Serial.print(F("MSG:INCONNU,"));
    Serial.println(cmd);
  }
}

// Lecture série non bloquante, ligne par ligne (pas de String :
// readStringUntil() de la v1 pouvait bloquer jusqu'à 1 s).
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0 && !rxDiscard) {
        rxBuf[rxLen] = '\0';
        handleCommand(rxBuf);
      }
      rxLen = 0;
      rxDiscard = false;
    } else if (rxDiscard) {
      // ligne trop longue : on jette jusqu'au prochain '\n'
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = toupper((unsigned char)c);
    } else {
      rxDiscard = true;
      rxLen = 0;
    }
  }
}

// ── Télémétrie 60 Hz ─────────────────────────────────────────────────
void emitTelemetry() {
  if ((long)(micros() - nextTelemetry) < 0) return;
  nextTelemetry += TELEMETRY_PERIOD_US;
  if ((long)(micros() - nextTelemetry) > (long)TELEMETRY_PERIOD_US)
    nextTelemetry = micros() + TELEMETRY_PERIOD_US;

  printPos();
  Serial.print(F(",CNT:"));
  Serial.print(encRead());
  Serial.print(F(",MIN:"));
  Serial.print(esMin.active ? 1 : 0);
  Serial.print(F(",MAX:"));
  Serial.print(esMax.active ? 1 : 0);
  Serial.print(F(",BRAKE:"));
  Serial.print(brakeReleased ? 1 : 0);
  Serial.print(F(",HOMED:"));
  Serial.println(isHomed ? 1 : 0);
}

// ── Machine à états principale ───────────────────────────────────────
void runState() {
  switch (state) {
    case ST_IDLE:
      break;

    case ST_PRE:                       // frein libéré, relais se stabilise
      if (millis() - preStart < DELAI_RELAIS_MS) break;
      switch (pending) {
        case PEND_MOVE:   planMove(pendTarget); break;
        case PEND_JOG:
          jogDir = pendJogDir;
          setDir(jogDir);
          supReset();
          nextStepDue = micros();
          state = ST_JOG;
          break;
        case PEND_VEL:
          velDir = pendVelDir;
          velPeriod = pendVelPeriod;
          setDir(velDir);
          supReset();
          nextStepDue = micros();
          velLastCmdMs = millis();   // fenêtre watchdog fraîche après le frein
          state = ST_VEL;
          break;
        case PEND_HOMING: homingStart(); state = ST_HOMING; break;
        default:          enterPost(); break;
      }
      pending = PEND_NONE;
      break;

    case ST_MOVE:
    case ST_CORRECT:
      if ((long)(micros() - nextStepDue) < 0) break;
      if (mv.dir == UP   && esMax.active) { fdcAbort(true);  break; }
      if (mv.dir == DOWN && esMin.active) { fdcAbort(false); break; }
      doPulse(mv.dir);
      mv.done++;
      if (mv.done >= mv.total) beginCorrectionOrFinish();
      else scheduleNext(state == ST_MOVE ? moveInterval() : mv.fast);
      break;

    case ST_JOG:
      if ((long)(micros() - nextStepDue) < 0) break;
      if (jogDir == UP   && esMax.active) { Serial.println(F("MSG:JOG_STOP")); fdcAbort(true);  break; }
      if (jogDir == DOWN && esMin.active) { Serial.println(F("MSG:JOG_STOP")); fdcAbort(false); break; }
      doPulse(jogDir);
      scheduleNext(PERIOD_JOG);
      break;

    case ST_VEL:
      // Watchdog homme-mort : sans nouveau VEL, on arrête + frein.
      if (millis() - velLastCmdMs > VEL_TIMEOUT_MS) {
        Serial.print(F("MSG:VEL_TIMEOUT,"));
        resyncOnEncoder();
        printPos();
        Serial.println();
        enterPost();
        break;
      }
      if ((long)(micros() - nextStepDue) < 0) break;
      if (velDir == UP   && esMax.active) { Serial.println(F("MSG:VEL_STOP")); fdcAbort(true);  break; }
      if (velDir == DOWN && esMin.active) { Serial.println(F("MSG:VEL_STOP")); fdcAbort(false); break; }
      doPulse(velDir);
      scheduleNext(velPeriod);
      break;

    case ST_HOMING:
      homingTick();
      break;

    case ST_POST:                      // moteur arrêté, on laisse le relais
      if (millis() - postStart < DELAI_RELAIS_MS) break;
      if (!manualBrake) setBrake(false);
      state = ST_IDLE;
      break;

    case ST_FDC:
      fdcTestTick();
      break;
  }
}

// ── Setup / Loop ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(PUL_PIN,     OUTPUT);
  pinMode(DIR_PIN,     OUTPUT);
  pinMode(ENA_PIN,     OUTPUT);
  pinMode(BRAKE_RELAY, OUTPUT);
  pinMode(ENDSTOP_MIN, INPUT_PULLUP);
  pinMode(ENDSTOP_MAX, INPUT_PULLUP);
  pinMode(ENC_A_PIN,   INPUT);         // sorties push-pull du codeur
  pinMode(ENC_B_PIN,   INPUT);

  digitalWrite(BRAKE_RELAY, LOW);      // frein serré au repos
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), isrEncoderA, CHANGE);

  updateEndstop(esMin);
  updateEndstop(esMax);

  Serial.println(F("MSG:BOOT,Chariot v2 boucle fermee"));
  Serial.println(F("MSG:PINOUT,PUL:D3,DIR:D4,ENA:D5,MIN:D8,MAX:D7,BRAKE:D10,ENCA:D2,ENCB:D6"));
  Serial.println(F("MSG:CMDS,H|POS:<mm>|VEL:<mm/s>|UP_START|UP_STOP|DOWN_START|DOWN_STOP|STOP|BRAKE:0/1|FDC"));

  nextTelemetry = micros() + TELEMETRY_PERIOD_US;
}

void loop() {
  pollSerial();          // 1. commandes (STOP traité immédiatement)
  updateEndstop(esMin);  // 2. fins de course (anti-rebond non bloquant)
  updateEndstop(esMax);
  runState();            // 3. avancement du mouvement en cours
  superviseStall();      // 4. supervision boucle fermée
  emitTelemetry();       // 5. télémétrie 60 Hz
}