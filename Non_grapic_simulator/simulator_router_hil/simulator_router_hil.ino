/*
 * Simulator — Router HIL (Hardware-In-the-Loop) variant
 * Router Arduino Due
 *
 * For the setup where STEERING is real (real motor + real angle potentiometer
 * read by DBW) but the vehicle's POSITION is simulated.
 *
 * Unlike simulator_closed_loop, this sketch does NOT simulate the wheel angle.
 * It TAKES the real angle that DBW measures from its pot (A10/A11 -> L_SENSE/
 * R_SENSE) and broadcasts on 0x400, and uses that real angle + the throttle to
 * dead-reckon the vehicle position. It then sends pose to the Sensor Hub so
 * Navigate has something to navigate by.
 *
 *   Inputs:
 *     THROTTLE_PIN (A0)  <- DBW throttle output (DAC0): drives simulated speed
 *     CAN 0x400 (Actual) <- DBW: bytes 0-1 speed, bytes 4-5 = REAL wheel angle (degx10)
 *   Outputs:
 *     IRPT_WHEEL_PIN (D47) -> DBW: simulated wheel-tick pulse (closes DBW speed loop)
 *     CAN to Sensor Hub:
 *       0x4C0 position (east_cm int32 + north_cm int32)
 *       0x4E0 heading  (heading_centiDeg int16)
 *       0x4F0 speed    (speed_cmPs int16)
 *
 * vs simulator_closed_loop: this sketch does NOT simulate the wheel angle, does
 * NOT drive L_SENSE/R_SENSE (the real pot owns DBW's A10/A11), and does NOT
 * publish 0x430. It KEEPS the wheel-tick pulse so DBW's speed PID still has
 * feedback. So: angle closed on the real pot, speed closed on simulated ticks.
 *
 * SETUP: Arduino IDE Sketchbook location -> <repo>/Non_grapic_simulator so it
 * finds the shared simulator_physics library.
 */

#include <SPI.h>
#include <SD.h>
#include <due_can.h>

#define USE_NATIVE_USB
#ifdef USE_NATIVE_USB
  #define LOG_PORT SerialUSB
#else
  #define LOG_PORT Serial
#endif

// ===== Pin Definitions =====
#define THROTTLE_PIN    A0     // From DBW DAC0 throttle output -> simulated speed
#define IRPT_WHEEL_PIN  47     // To DBW: simulated wheel-tick pulse (closes DBW speed loop)
// L_SENSE/R_SENSE are NOT driven here — in HIL the real pot drives DBW's A10/A11.
// Kept defined only because the shared physics library references them.
#define L_SENSE_PIN     DAC0
#define R_SENSE_PIN     DAC1

#define SD_CS_PIN       37

// ===== CAN ID Definitions =====
#define Actual_CANID      0x400   // DBW->bus: actual speed (0-1) + real angle (4-5)
#define CAN_POSITION      0x4C0   // Router->SH: east_cm int32 + north_cm int32
#define CAN_HEADING       0x4E0   // Router->SH: heading_centiDeg int16
#define CAN_SPEED         0x4F0   // Router->SH: speed_cmPs int16

// ===== Speed Model Constants =====
#define FRICTION_NUM            9296
#define FRICTION_DEN            10000
#define MIN_EFFECTIVE_THROTTLE  65
#define MAX_EFFECTIVE_THROTTLE  227
#define MAX_SPEED_mmPs          2000
#define THROTTLE_HISTORY        10
#define THROTTLE_DELAY_START    3
#define THROTTLE_DELAY_END      10

// ===== Vehicle Settings =====
#define WHEEL_DIAMETER_MM    495
#define WHEEL_CIRCUM_MM      1555
#define LOOP_TIME_MS         100
#define MAX_ANGLE_TENTHS     250
#define ANGLE_CHANGE_TENTHS  20
#define L_STRAIGHT  722
#define L_MIN       779
#define L_MAX       639
#define R_STRAIGHT  731
#define R_MIN       673
#define R_MAX       786

// ===== Global Variables (all integer) =====
int throttleHistory[THROTTLE_HISTORY];
int historyIndex = 0;

int speed_mmPs     = 0;
int prevSpeed_mmPs = 0;
int heading_tenths = 0;
int angle_tenths   = 0;     // REAL wheel angle, taken from DBW's 0x400 (not simulated)

long X_mm = 0;
long Y_mm = 0;

unsigned long nextPulseTime_ms = 0;

File logFile;
bool sdAvailable = false;

// ===== Function Declarations =====
bool sendPositionCAN();
bool sendHeadingCAN();
bool sendSpeedCAN();
void drainCanRx();

#include <simulator_physics.h>

// ===========================================================================
void setup() {
  LOG_PORT.begin(115200);
#ifdef USE_NATIVE_USB
  uint32_t waitStart = millis();
  while (!LOG_PORT && (millis() - waitStart) < 3000);
#endif

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(IRPT_WHEEL_PIN, OUTPUT);
  digitalWrite(IRPT_WHEEL_PIN, LOW);
  for (int i = 0; i < THROTTLE_HISTORY; i++) throttleHistory[i] = 0;

  // CAN at 500 kbps. This sketch RECEIVES 0x400 from DBW, so install a
  // catch-all RX filter (simulator_closed_loop was transmit-only).
  if (Can0.begin(CAN_BPS_500K)) {
    Can0.watchFor();
    LOG_PORT.println("CAN init OK");
  } else {
    LOG_PORT.println("CAN init FAILED");
  }

  // SD diagnostic log (optional)
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);
  if (SD.begin(SD_CS_PIN)) {
    char filename[13];
    for (int i = 0; i < 100; i++) {
      sprintf(filename, "HIL%02d.CSV", i);
      if (!SD.exists(filename)) break;
    }
    logFile = SD.open(filename, FILE_WRITE);
    if (logFile) {
      sdAvailable = true;
      logFile.println("time_ms,X_mm,Y_mm,heading_tenths,speed_mmPs,angle_tenths");
      logFile.flush();
    }
  }

  LOG_PORT.println("Router HIL (real angle from DBW 0x400) started.");
}

// ===========================================================================
void loop() {
  uint32_t startTime = millis();

  // 1. Get the REAL wheel angle from DBW (0x400). Sets angle_tenths directly —
  //    no slew, no physics on the angle. The pot is the source of truth.
  drainCanRx();

  // 2. Simulated speed from DBW's throttle output (analog pin). No real wheel
  //    is turning on the bench, so we model speed from the throttle.
  int rawThrottle = analogRead(THROTTLE_PIN);
  int throttle = rawThrottle / 4;
  bool brakeOn = (throttle < MIN_EFFECTIVE_THROTTLE);

  throttleHistory[historyIndex] = throttle;
  historyIndex = (historyIndex + 1) % THROTTLE_HISTORY;
  speed_mmPs   = computeSpeed(throttle, brakeOn);

  // 2b. Send simulated wheel-tick pulses to DBW so its speed PID has feedback
  //     (closes DBW's speed loop on the simulated speed). We do NOT drive
  //     L_SENSE/R_SENSE — the real steering pot owns those.
  sendWheelPulse(speed_mmPs);

  // 3. Dead-reckon position using the REAL angle + simulated speed.
  updatePosition(speed_mmPs, heading_tenths, angle_tenths);

  // 4. Publish pose to the Sensor Hub.
  sendPositionCAN();
  sendHeadingCAN();
  sendSpeedCAN();

  static uint32_t lastDbg_ms = 0;
  if (millis() - lastDbg_ms > 1000) {
    lastDbg_ms = millis();
    LOG_PORT.print("# HIL angle="); LOG_PORT.print(angle_tenths);
    LOG_PORT.print(" speed=");      LOG_PORT.print(speed_mmPs);
    LOG_PORT.print(" east_cm=");    LOG_PORT.print(X_mm / 10);
    LOG_PORT.print(" north_cm=");   LOG_PORT.print(Y_mm / 10);
    LOG_PORT.print(" heading=");    LOG_PORT.println(heading_tenths);
  }

  if (sdAvailable) {
    logFile.print(millis());       logFile.print(",");
    logFile.print(X_mm);           logFile.print(",");
    logFile.print(Y_mm);           logFile.print(",");
    logFile.print(heading_tenths); logFile.print(",");
    logFile.print(speed_mmPs);     logFile.print(",");
    logFile.println(angle_tenths);
    logFile.flush();
  }

  uint32_t elapsed = millis() - startTime;
  if (elapsed < LOOP_TIME_MS) delay(LOOP_TIME_MS - elapsed);
}

// ===========================================================================
// Take the REAL wheel angle from DBW's 0x400 (bytes 4-5). No simulation —
// angle_tenths becomes exactly what DBW's pot measured.
void drainCanRx() {
  CAN_FRAME f;
  while (Can0.available() > 0) {
    Can0.read(f);
    if (f.id == Actual_CANID && f.length >= 6) {
      angle_tenths = (int16_t)(f.data.bytes[4] | (f.data.bytes[5] << 8));
    }
  }
}

// ===========================================================================
// CAN senders (Router -> Sensor Hub). due_can BytesUnion: value/low,high/
// s0..s3/bytes[8]; little-endian on the wire.
bool sendPositionCAN() {
  CAN_FRAME f;
  f.id = CAN_POSITION; f.extended = false; f.length = 8;
  f.data.low  = (uint32_t)(int32_t)(X_mm / 10);   // east_cm
  f.data.high = (uint32_t)(int32_t)(Y_mm / 10);   // north_cm
  return Can0.sendFrame(f);
}

bool sendHeadingCAN() {
  CAN_FRAME f;
  f.id = CAN_HEADING; f.extended = false; f.length = 2;
  f.data.s0 = (uint16_t)(int16_t)((long)heading_tenths * 10);
  return Can0.sendFrame(f);
}

bool sendSpeedCAN() {
  CAN_FRAME f;
  f.id = CAN_SPEED; f.extended = false; f.length = 2;
  f.data.s0 = (uint16_t)(int16_t)(speed_mmPs / 10);
  return Can0.sendFrame(f);
}
