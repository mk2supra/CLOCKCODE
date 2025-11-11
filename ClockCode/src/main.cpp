// IRL CLOCK
#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>

#define DRIVER 1
#define hourStepPin 9
#define hourDirPin 10
#define minStepPin 11
#define minDirectionPin 12
#define hourLimSwitch 13
#define minLimSwitch 5

// steppers
AccelStepper stepperHour(DRIVER, hourStepPin, hourDirPin);
AccelStepper stepperMin (DRIVER, minStepPin, minDirectionPin);

// step parameters
const int  hourStepDirect = +1; //motordirecyion
const int  minStepDirect = +1;
const bool hourlimswitch = true;
const bool minlimswitch = true;

const long stepsPerRev = 447;//approx step count for 1 rotation

const float hourspeed = 200;//steps/sec during home()
const float minspeed = 200;

// true if switch is pressed
bool limitIsActive(int pin, bool activeHigh) {
  int v = digitalRead(pin);
  return activeHigh ? (v == HIGH) : (v == LOW);
}

// WiFi
const char* WIFI_SSID = "NSA Security Van HQ";
const char* WIFI_PW   = "windowstothehallway";
const char* NTP_SERVER= "pool.ntp.org";
const long  gmtOffset_sec = -21600; // time shift 2 work
const int   daylightOffset_sec = 0;

// thing that makes it move only clockwise
static inline long modWrap(long x, long m) {
  long r = x % m;
  return (r < 0) ? (r + m) : r;
}
long forwardOnlyWrappedTarget(long currentAbs, long targetMod, long stepsPerRev) {
  long curMod = modWrap(currentAbs, stepsPerRev);
  long forward = targetMod - curMod;
  if (forward < 0) forward += stepsPerRev;// never turn backward
  return currentAbs + forward;//absolute target ahead
}

// home function, homes hour ring first then the min gear, it could probably do both simultainously now my wiring changes but I'll leave it
void home() {
  Serial.println("Homing hour...");
  stepperHour.setMaxSpeed(hourspeed);
  stepperHour.setSpeed(hourspeed * hourStepDirect);
  while (!limitIsActive(hourLimSwitch, hourlimswitch)) {
    stepperHour.runSpeed();
  }
  stepperHour.stop();
  stepperHour.setCurrentPosition(0);
  Serial.println("Hour homed.");

  delay(500);

  Serial.println("Homing minute...");
  stepperMin.setMaxSpeed(minspeed);
  stepperMin.setSpeed(minspeed * minStepDirect);
  while (!limitIsActive(minLimSwitch, minlimswitch)) {
    stepperMin.runSpeed();
  }
  stepperMin.stop();
  stepperMin.setCurrentPosition(0);
  Serial.println("Minute homed.");
}

//connetct to wifi
void connectWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PW);
  while (WiFi.status() != WL_CONNECTED) { delay(150); Serial.print("."); }
  Serial.println("\nConnected!");
  configTime(gmtOffset_sec, daylightOffset_sec, NTP_SERVER);
}

//read time and move ring gears
void timeUpdate() {
  struct tm t;
  if (!getLocalTime(&t)) { Serial.println("Failed to get time"); return; }

  char timedisp[10];
  strftime(timedisp, sizeof(timedisp), "%H:%M:%S", &t);
  Serial.print("Local time: ");
  Serial.println(timedisp);

  int hour12 = t.tm_hour % 12;
  int minute = t.tm_min;
  int second = t.tm_sec;

  // Hour gear math
  double hourSteps =(hour12 * (double)stepsPerRev / 12.0) +(minute * ((double)stepsPerRev / 720.0))+(second * ((double)stepsPerRev / 43200.0));

  // Minute math
  double minSteps=(minute * ((double)stepsPerRev / 60.0)) + (second * ((double)stepsPerRev / 3600.0));

  long hourTargetMod = lround(hourSteps) % stepsPerRev;
  long minTargetMod  = lround(minSteps)  % stepsPerRev;
  if (hourTargetMod < 0) hourTargetMod += stepsPerRev;
  if (minTargetMod  < 0) minTargetMod  += stepsPerRev;

  long hourAbs = forwardOnlyWrappedTarget(stepperHour.currentPosition(),hourTargetMod, stepsPerRev);
  long minAbs  = forwardOnlyWrappedTarget(stepperMin.currentPosition(), minTargetMod,  stepsPerRev);

  stepperHour.moveTo(hourAbs);
  stepperMin.moveTo(minAbs);
}

void setup() {
  Serial.begin(115200);

  pinMode(hourLimSwitch, INPUT_PULLUP);
  pinMode(minLimSwitch, INPUT_PULLUP);

  // speed change to tune IRL stuffs
  stepperHour.setMinPulseWidth(5);
  stepperMin.setMinPulseWidth(5);
  stepperHour.setAcceleration(100);
  stepperMin.setAcceleration(100);
  stepperHour.setMaxSpeed(120);
  stepperMin.setMaxSpeed(120);

//first home then wifi then update time
  home();
  connectWiFi();
  timeUpdate();
}

void loop() {
  static unsigned long lastSync = 0;
  unsigned long now = millis();

  //update the time oncer per sec
  if (now - lastSync > 1000) {
    lastSync = now;
    timeUpdate();
  }
//tell the steps to move
  stepperHour.run();
  stepperMin.run();
}