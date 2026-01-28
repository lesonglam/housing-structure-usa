#include <SPI.h>
#include <SoftwareSerial.h>
#include <avr/interrupt.h>
#include "U8glib.h"

// ---------------- Pins ----------------
#define RLP1  A0
#define RLP2  A1
#define RLP3  A2
#define RLP4  A3

#define AD1   A4   // SDA (analog in)
#define AD2   A5   // SCL (analog in)

// Optional: keep if you need it for other hardware
SoftwareSerial mySerial(7, 8);

// LCD
U8GLIB_ST7920_128X64_4X u8g(6, 5, 4);

// --------- Actuator calibration ----------
const int STROKE_IN_X100 = 1000;   // 10.00 inches = 1000 (x100)

// Set these after measuring
int adcMin = 0;       // ADC at fully retracted
int adcMax = 1023;    // ADC at fully extended

// ---------------- Command / State ----------------
enum DirState { DIR_STOP = 0, DIR_FWR = 1, DIR_REV = 2 };
DirState dirState = DIR_STOP;

// Serial parsing buffer
static char cmdBuf[32];
static uint8_t cmdLen = 0;

// Telemetry rate (ms)
const unsigned long TELEMETRY_PERIOD_MS = 100;
unsigned long lastTelemetryMs = 0;

// ---------------- Helpers ----------------
void setRelays(bool on) {
  digitalWrite(RLP1, on ? HIGH : LOW);
  digitalWrite(RLP2, on ? HIGH : LOW);
  digitalWrite(RLP3, on ? HIGH : LOW);
  digitalWrite(RLP4, on ? HIGH : LOW);
}

// NOTE: You MUST map these to your actual relay wiring.
// Right now: STOP = all relays OFF.
// FWR/REV = placeholders that turn some relays ON/OFF.
// Adjust to match how your H-bridge / relays are wired.
void applyDirection(DirState s) {
  dirState = s;

  switch (s) {
    case DIR_STOP:
      // All OFF = stop
      setRelays(false);
      break;

    case DIR_FWR:
      // Placeholder logic: adjust for your relay wiring
      // Example: RLP1+RLP2 ON, RLP3+RLP4 OFF
      digitalWrite(RLP3, HIGH);
      digitalWrite(RLP4, LOW); 
      break;

    case DIR_REV:
      // Placeholder logic: adjust for your relay wiring
      // Example: RLP1+RLP2 OFF, RLP3+RLP4 ON
      digitalWrite(RLP3, LOW);
      digitalWrite(RLP4, HIGH);
     
      break;
  }
}

int readAvg(int pin, int n = 10) {
  long sum = 0;
  for (int i = 0; i < n; i++) sum += analogRead(pin);
  return (int)(sum / n);
}

int adcToMilliVolts(int adc) {
  return (long)adc * 5000L / 1023L; // 0..1023 -> 0..5000 mV
}

int adcToInchesX100(int adc) {
  if (adc < adcMin) adc = adcMin;
  if (adc > adcMax) adc = adcMax;

  long den = (long)(adcMax - adcMin);
  if (den <= 0) return 0;

  long num = (long)(adc - adcMin) * (long)STROKE_IN_X100;
  return (int)(num / den); // inches * 100
}

const char* dirToStr(DirState s) {
  switch (s) {
    case DIR_FWR:  return "FWR";
    case DIR_REV:  return "REV";
    default:       return "STOP";
  }
}

// Single easy-to-parse line (CSV) for Python
// Format:
// T,<DIR>,A1=<adc>,IN1=<in_x100>,MV1=<mv>,A2=<adc>,IN2=<in_x100>,MV2=<mv>
void printTelemetryLine(int adc1, int adc2) {
  int mv1 = adcToMilliVolts(adc1);
  int mv2 = adcToMilliVolts(adc2);
  int in1x100 = adcToInchesX100(adc1);
  int in2x100 = adcToInchesX100(adc2);

  Serial.print("T,");
  Serial.print(dirToStr(dirState));
  Serial.print(",A1=");
  Serial.print(adc1);
  Serial.print(",IN1=");
  Serial.print(in1x100);
  Serial.print(",MV1=");
  Serial.print(mv1);
  Serial.print(",A2=");
  Serial.print(adc2);
  Serial.print(",IN2=");
  Serial.print(in2x100);
  Serial.print(",MV2=");
  Serial.println(mv2);
}

// LCD: show direction + either ADC or inches
void drawDisplay(int adc1, int adc2) {
  int in1x100 = adcToInchesX100(adc1);
  int in2x100 = adcToInchesX100(adc2);

  char header[22];
  char line1[22];
  char line2[22];

  snprintf(header, sizeof(header), "DIR: %s", dirToStr(dirState));

  // You can swap to raw ADC if you prefer
  // Example shows inches
  snprintf(line1, sizeof(line1), "P1:%d.%02din A:%d",
           in1x100 / 100, in1x100 % 100, adc1);

  snprintf(line2, sizeof(line2), "P2:%d.%02din A:%d",
           in2x100 / 100, in2x100 % 100, adc2);

  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_6x12);
    u8g.drawStr(0, 14, "Actuator Telemetry");
    u8g.drawStr(0, 30, header);
    u8g.drawStr(0, 46, line1);
    u8g.drawStr(0, 62, line2);
  } while (u8g.nextPage());
}

// Handle a complete command line (trimmed, uppercased)
// Commands:
//   FWR
//   REV
//   STOP
// Optional:
//   READ   (prints one telemetry line immediately)
void handleCommand(char* cmd) {
  // Uppercase in-place
  for (uint8_t i = 0; cmd[i]; i++) {
    if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] = cmd[i] - 'a' + 'A';
  }

  if (strcmp(cmd, "FWR") == 0) {
    applyDirection(DIR_FWR);
    Serial.println("OK,FWR");
  } else if (strcmp(cmd, "REV") == 0) {
    applyDirection(DIR_REV);
    Serial.println("OK,REV");
  } else if (strcmp(cmd, "STOP") == 0) {
    applyDirection(DIR_STOP);
    Serial.println("OK,STOP");
  } else if (strcmp(cmd, "READ") == 0) {
    int adc1 = readAvg(AD1);
    int adc2 = readAvg(AD2);
    printTelemetryLine(adc1, adc2);
  } else {
    Serial.print("ERR,UNKNOWN_CMD,");
    Serial.println(cmd);
  }
}

// Non-blocking serial line reader. Commands end with '\n' or '\r'.
void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else {
      if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      } else {
        // overflow -> reset
        cmdLen = 0;
        Serial.println("ERR,BUF_OVERFLOW");
      }
    }
  }
}

void setup() {
  pinMode(RLP1, OUTPUT);
  pinMode(RLP2, OUTPUT);
  pinMode(RLP3, OUTPUT);
  pinMode(RLP4, OUTPUT);

  applyDirection(DIR_STOP);

  Serial.begin(9600);
  mySerial.begin(9600); // optional; not used below

  Serial.println("READY,LinearActuator");
  Serial.println("CMDS:FWR|REV|STOP|READ");
}

void loop() {
  // 1) Receive commands (non-blocking)
  pollSerialCommands();

  // 2) Read sensors
  int adc1 = readAvg(AD1);
  int adc2 = readAvg(AD2);

  // 3) LCD display
  drawDisplay(adc1, adc2);

  // 4) Periodic telemetry stream (easy for Python to parse)
  unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    printTelemetryLine(adc1, adc2);
  }
}
