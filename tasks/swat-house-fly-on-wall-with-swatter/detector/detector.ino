// Fly escape detector for the swat task (Arduino Nano, 5 V; it uses A6 and A7, which an Uno does not have).
//
// Rule (the same as the simulator's): once anything comes within 150 mm of the fly, the fly leaves 100 ms later, whatever
// happens in between (even if the swatter is pulled back). Here "within 150 mm" is read by 13 VL53L1X time-of-flight
// sensors set flush in the wall round the fly (layout and thresholds: WIRING.md, coverage.py). When the 100 ms are up the
// solenoid pulls the inked fly pad 2.5 mm behind the wall face, and the red ESCAPED LED latches on until RESET is pressed.
// A swat is read afterwards from the pad's ink on the swatter's head: the pad is only out to be touched until the fly leaves.
//
// Library: "VL53L1X" by Pololu (Arduino Library Manager), https://github.com/pololu/vl53l1x-arduino
//
// Modes: normal (CAL jumper open) and calibration (CAL jumper, A2 to GND): in calibration the solenoid never fires; the
// ESCAPED LED is lit while any sensor is under its threshold, and every reading is printed at 115200 baud as
// "ms S0 S1 ... S12 near" (mm, 0 = no valid target), for the procedure in CALIBRATION.md.

#include <Wire.h>
#include <VL53L1X.h>

const uint8_t N = 13;
const uint8_t XSHUT[N] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 12, A0, A1, A2};  // S0..S12
// Range thresholds (mm from each sensor's window), from coverage.py: a target on a sensor's axis fires at 150 mm from the
// fly's centre; S4 and S12 look partly down and stay 15 mm short of the table top in their cones. If CALIBRATION.md step 2
// finds a sensor reading the wall or the table in an empty scene, lower its threshold there until it is quiet.
const uint16_t THRESH_MM[N] = {149, 134, 134, 134, 123, 116, 116, 116, 116, 111, 111, 111, 85};

const uint8_t PIN_SOLENOID = 11;  // logic-level MOSFET gate (PWM pin)
const uint8_t PIN_ESCAPED = 13;   // red LED (and the Nano's own LED): the fly has left (latched)
const uint8_t PIN_ARMED = A3;     // green LED: armed, fly out
const uint8_t PIN_RESET = A6;     // push button to GND, 10 k pull-up to 5V (A6 is analog input only)
const uint8_t PIN_CAL = A7;       // jumper to GND, 10 k pull-up to 5V (analog input only): calibration mode

bool low(uint8_t analogOnlyPin) { return analogRead(analogOnlyPin) < 300; }

const uint32_t ESCAPE_MS = 100;     // the simulator's 0.1 s (two 20 Hz checks)
const uint32_t PULL_MS = 150;       // full power to pull the pad in
const uint8_t HOLD_PWM = 110;       // then about 43 % to hold it in without overheating the coil
const uint32_t HOLD_MAX_MS = 180000;  // let the pad out again after 3 minutes if nobody pressed RESET (the LED stays on)

VL53L1X sensor[N];
uint16_t last[N];

enum State { ARMED, TIMING, ESCAPED };
State state = ARMED;
uint32_t t_near = 0, t_fire = 0;
bool cal = false;

void fail(uint8_t i) {  // a sensor did not start: blink its number on the red LED forever
  for (;;) {
    for (uint8_t k = 0; k <= i; k++) {
      digitalWrite(PIN_ESCAPED, HIGH); delay(200);
      digitalWrite(PIN_ESCAPED, LOW); delay(200);
    }
    delay(1000);
  }
}

void setup() {
  pinMode(PIN_SOLENOID, OUTPUT); analogWrite(PIN_SOLENOID, 0);
  pinMode(PIN_ESCAPED, OUTPUT); pinMode(PIN_ARMED, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  // Hold every sensor in shutdown (XSHUT is not 5 V tolerant: drive it low, never high; the carrier pulls it up to 2.8 V).
  for (uint8_t i = 0; i < N; i++) { pinMode(XSHUT[i], OUTPUT); digitalWrite(XSHUT[i], LOW); }
  delay(10);
  // Wake them one at a time and give each its own I2C address.
  for (uint8_t i = 0; i < N; i++) {
    pinMode(XSHUT[i], INPUT);
    delay(10);
    sensor[i].setTimeout(100);
    if (!sensor[i].init()) fail(i);
    sensor[i].setAddress(0x30 + i);
    sensor[i].setDistanceMode(VL53L1X::Short);
    sensor[i].setMeasurementTimingBudget(20000);  // 20 ms
    sensor[i].startContinuous(20);                // a new reading from every sensor every 20 ms (50 Hz)
    last[i] = 0;
  }
  cal = low(PIN_CAL);
  digitalWrite(PIN_ARMED, HIGH);
  Serial.println(cal ? F("# calibration mode: solenoid disabled") : F("# armed"));
}

bool readNear() {
  bool near = false;
  for (uint8_t i = 0; i < N; i++) {
    if (sensor[i].dataReady()) {
      uint16_t r = sensor[i].read(false);
      last[i] = (sensor[i].ranging_data.range_status == VL53L1X::RangeValid) ? r : 0;
    }
    if (last[i] > 0 && last[i] < THRESH_MM[i]) near = true;
  }
  return near;
}

void loop() {
  uint32_t now = millis();
  bool near = readNear();

  if (cal) {
    digitalWrite(PIN_ESCAPED, near ? HIGH : LOW);
    Serial.print(now);
    for (uint8_t i = 0; i < N; i++) { Serial.print(' '); Serial.print(last[i]); }
    Serial.print(' '); Serial.println(near ? 1 : 0);
    return;
  }

  switch (state) {
    case ARMED:
      if (near) { state = TIMING; t_near = now; }  // the timer starts at the first reading under a threshold
      break;
    case TIMING:  // it runs whatever the sensors read next: pulling back does not stop it
      if (now - t_near >= ESCAPE_MS) {
        analogWrite(PIN_SOLENOID, 255);
        digitalWrite(PIN_ESCAPED, HIGH);
        digitalWrite(PIN_ARMED, LOW);
        t_fire = now;
        state = ESCAPED;
        Serial.print(F("# escaped at ")); Serial.println(now);
      }
      break;
    case ESCAPED:
      if (now - t_fire >= HOLD_MAX_MS) analogWrite(PIN_SOLENOID, 0);
      else if (now - t_fire >= PULL_MS) analogWrite(PIN_SOLENOID, HOLD_PWM);
      break;
  }

  if (low(PIN_RESET) && !near) {  // RESET: pad out, LED off, re-armed (only with nothing near)
    analogWrite(PIN_SOLENOID, 0);
    digitalWrite(PIN_ESCAPED, LOW);
    digitalWrite(PIN_ARMED, HIGH);
    state = ARMED;
    delay(300);
  }
}
