#include <Arduino.h>
#include <ESP32Encoder.h>
#include <DRV8833.h>
#include <PID_v1.h>

// --- CONFIGURATION ---
const int PPR_A = 2520; 
const int PPR_B = 2530;

const unsigned long PID_SAMPLE_TIME = 20;   // Calcul PID toutes les 20ms (très réactif)
const unsigned long SERIAL_INTERVAL = 1000; // Affichage toutes les 1000ms

// Pins DRV8833
const int MOTOR_A_IN1 = 12;
const int MOTOR_A_IN2 = 13;
const int MOTOR_B_IN1 = 14;
const int MOTOR_B_IN2 = 27;

double Kp = 2.0, Ki = 5.0, Kd = 0.1;

// --- CLASSES ---
class Senseur {
  private:
    int _pin;
  public:
    Senseur(int pin) : _pin(pin) {}
    void begin() { pinMode(_pin, INPUT_PULLUP); }
    bool estActif() { return digitalRead(_pin) == LOW; }
};

class Potentiometre {
  private:
    int _pin;
    int _minOut, _maxOut;
    float _valeurLissee;
    float _alpha;
  public:
    Potentiometre(int pin, int minOut, int maxOut, float alpha = 0.15) 
      : _pin(pin), _minOut(minOut), _maxOut(maxOut), _alpha(alpha) {
      _valeurLissee = 2048;
    }
    void actualiser() {
      _valeurLissee = (_alpha * analogRead(_pin)) + ((1.0 - _alpha) * _valeurLissee);
    }
    int lire() {
      return constrain(map((int)_valeurLissee, 0, 4095, _minOut, _maxOut), _minOut, _maxOut);
    }
};

// --- INSTANCES ---
ESP32Encoder encoderA;
ESP32Encoder encoderB;
Potentiometre monPot(34, -255, 255, 0.1); 
Senseur boutonUrgence(33);

DRV8833 moteurA(MOTOR_A_IN1, MOTOR_A_IN2, -255, 255, 10, false, false);
DRV8833 moteurB(MOTOR_B_IN1, MOTOR_B_IN2, -255, 255, 10, false, false);

double consigneRPM_A = 0, vitesseActuelle_A = 0, commandePWM_A = 0;
double consigneRPM_B = 0, vitesseActuelle_B = 0, commandePWM_B = 0;

PID pidMoteurA(&vitesseActuelle_A, &commandePWM_A, &consigneRPM_A, Kp, Ki, Kd, DIRECT);
PID pidMoteurB(&vitesseActuelle_B, &commandePWM_B, &consigneRPM_B, Kp, Ki, Kd, DIRECT);

unsigned long precedentMillisPID = 0;
unsigned long precedentMillisSerial = 0;

int64_t posA = 0;
int64_t posB = 0;

int64_t anciennePosA = 0;
int64_t anciennePosB = 0;

bool urgenceActive = false;
const float RPM_MAX = 255.0;

void setup() {
  Serial.begin(115200);
  boutonUrgence.begin();
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoderA.attachFullQuad(18, 19);
  encoderB.attachFullQuad(16, 17);

  pidMoteurA.SetMode(AUTOMATIC);
  pidMoteurA.SetOutputLimits(-255, 255);
  pidMoteurA.SetSampleTime(PID_SAMPLE_TIME);
  
  pidMoteurB.SetMode(AUTOMATIC);
  pidMoteurB.SetOutputLimits(-255, 255);
  pidMoteurB.SetSampleTime(PID_SAMPLE_TIME);

  moteurA.drive(0, 255, 0, true, true);
  moteurB.drive(0, 255, 0, true, true);
}

void loop() {
  unsigned long actuelMillis = millis();

  
  // 1. GESTION CONTINUE (Potentiomètre et Bouton)
  monPot.actualiser();
  int vitesseCible = monPot.lire();
  double consigneRPM = map(vitesseCible, -255, 255, -RPM_MAX, RPM_MAX);
  consigneRPM_A = consigneRPM;
  consigneRPM_B = consigneRPM;

  if (boutonUrgence.estActif()) {
      urgenceActive = true;
      pidMoteurA.SetMode(MANUAL);
      pidMoteurB.SetMode(MANUAL);
      commandePWM_A = 0; commandePWM_B = 0;
      moteurA.drive(0, 255, 0, true, true);
      moteurB.drive(0, 255, 0, true, true);
  } else {
    pidMoteurA.SetMode(AUTOMATIC);
    pidMoteurB.SetMode(AUTOMATIC);

        // 2. LOGIQUE DE CALCUL (PID) - Fréquence Rapide
    if (actuelMillis - precedentMillisPID >= PID_SAMPLE_TIME) {
      posA = encoderA.getCount();
      posB = encoderB.getCount();

      // Vitesse en RPM
      vitesseActuelle_A = (float(posA - anciennePosA) / PPR_A) * (1000.0 / PID_SAMPLE_TIME) * 60.0;
      vitesseActuelle_B = (float(posB - anciennePosB) / PPR_B) * (1000.0 / PID_SAMPLE_TIME) * 60.0;

      pidMoteurA.Compute();
      pidMoteurB.Compute();
      moteurA.drive((int)commandePWM_A, 255, 0, false, false);
      moteurB.drive((int)commandePWM_B, 255, 0, false, false);

      anciennePosA = posA;
      anciennePosB = posB;
      precedentMillisPID = actuelMillis;
    }

  }


  // 3. AFFICHAGE (Serial) - Fréquence Lente (1 seconde)
  if (actuelMillis - precedentMillisSerial >= SERIAL_INTERVAL) {
    Serial.printf("%d, %.1f, %d, %.1f, %d, %d, %d\n", 
                  vitesseCible, vitesseActuelle_A, (int)commandePWM_A, 
                  vitesseActuelle_B, (int)commandePWM_B, posA, posB);
    precedentMillisSerial = actuelMillis;
  }
}
