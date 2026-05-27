#include <SPI.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ISD_SS 10
#define ISD2_SS 9
#define PTT 6
#define ROGER 7
#define CHANNEL_UP 3
#define SQL 5
#define BATTERY_PIN A5

#define DTMF_STD A0
#define DTMF_Q4 A1
#define DTMF_Q3 A2
#define DTMF_Q2 A3
#define DTMF_Q1 A4
#define TEMP_PIN 2

const uint32_t SPI_SPEED = 10000;

const uint16_t RECORD_START_ADDR = 0x010;
const uint16_t MAX_END_ADDR      = 0x78F;

const uint32_t MAX_RECORD_MS = 150000UL;
const uint32_t GLOBAL_ERASE_WAIT_MS = 500UL;

const uint32_t SQL_START_STABLE_MS = 100UL;
const uint32_t SQL_STOP_STABLE_MS = 1000UL;

const uint32_t PAUSE_AFTER_RX_MS = 500UL;
const uint32_t PAUSE_BEFORE_PLAY_MS = 1000UL;
const uint32_t PAUSE_AFTER_PTT_OFF_MS = 500UL;

const uint32_t BATTERY_MEASURE_BEFORE_END_MS = 100UL;
const float FALLBACK_BATTERY_CONST = 0.014632;
const float LOW_BATTERY_LIMIT = 11.0;

const uint32_t DTMF_DIGIT_TIMEOUT_MS = 2000UL;
const uint32_t DTMF_STABLE_MS = 40UL;
const uint32_t DTMF_PRINT_AFTER_LAST_MS = 3000UL;
const uint8_t DTMF_CODE_LEN = 5;
const uint8_t DTMF_TONE_LOG_MAX_LEN = 16;
const char DTMF_CODE_ON[DTMF_CODE_LEN + 1] = "A3279";
const char DTMF_CODE_OFF[DTMF_CODE_LEN + 1] = "D8133";
const char DTMF_CODE_ROGER_OFF[DTMF_CODE_LEN + 1] = "C4280";
const char DTMF_CODE_ROGER_1[DTMF_CODE_LEN + 1] = "C4281";
const char DTMF_CODE_ROGER_2[DTMF_CODE_LEN + 1] = "C4282";
const char DTMF_CODE_ROGER_3[DTMF_CODE_LEN + 1] = "C4283";
const char DTMF_CODE_ROGER_4[DTMF_CODE_LEN + 1] = "C4284";
const char DTMF_CODE_ROGER_5[DTMF_CODE_LEN + 1] = "C4285";
const char DTMF_CODE_ROGER_6[DTMF_CODE_LEN + 1] = "C4286";
const char DTMF_CODE_ROGER_7[DTMF_CODE_LEN + 1] = "C4287";
const char DTMF_CODE_ROGER_ALL[DTMF_CODE_LEN + 1] = "C4289";
const char DTMF_CODE_CHANNEL_UP_1[DTMF_CODE_LEN + 1] = "72961";
const char DTMF_CODE_CHANNEL_UP_2[DTMF_CODE_LEN + 1] = "72962";
const char DTMF_CODE_CHANNEL_UP_3[DTMF_CODE_LEN + 1] = "72963";
const char DTMF_CODE_CHANNEL_UP_4[DTMF_CODE_LEN + 1] = "72964";
const char DTMF_CODE_CHANNEL_UP_5[DTMF_CODE_LEN + 1] = "72965";
const char DTMF_CODE_STATUS_REPORT[DTMF_CODE_LEN + 1] = "11025";
const char DTMF_CODE_AUTO_STATUS_OFF[DTMF_CODE_LEN + 1] = "11020";
const char DTMF_CODE_AUTO_STATUS_1H[DTMF_CODE_LEN + 1] = "11021";
const char DTMF_CODE_AUTO_STATUS_4H[DTMF_CODE_LEN + 1] = "11022";
const char DTMF_CODE_AUTO_STATUS_12H[DTMF_CODE_LEN + 1] = "11023";
const char DTMF_CODE_AUTO_STATUS_24H[DTMF_CODE_LEN + 1] = "11024";

const uint8_t ROGER_OFF = 0;
const uint8_t ROGER_DEFAULT = 1;
const uint8_t ROGER_COUNT = 7;
const uint32_t ROGER_ALL_PAUSE_MS = 2000UL;
const uint32_t CHANNEL_UP_PULSE_MS = 100UL;
const uint32_t CHANNEL_UP_PAUSE_MS = 400UL;

const uint8_t ISD1_ROSC_KOHM = 53;
const uint8_t ISD2_ROSC_KOHM = 80;
const uint32_t ISD1_ROW_TIME_MS = 85UL;
const uint32_t ISD2_ROW_TIME_MS = (ISD1_ROW_TIME_MS * ISD2_ROSC_KOHM) / ISD1_ROSC_KOHM;

const uint32_t ISD2_CLIP_PAUSE_MS = 10UL;
const uint32_t STATUS_REPORT_PAUSE_MS = 200UL;
const uint32_t ISD2_READY_TIMEOUT_MS = 10000UL;

const uint32_t AUTO_STATUS_IDLE_MS = 60000UL;
const uint32_t AUTO_STATUS_DEFER_MS = 120000UL;

const uint16_t EEPROM_BATTERY_CONST_ADDR = 0;
const uint16_t EEPROM_REPEATER_STATE_ADDR = EEPROM_BATTERY_CONST_ADDR + sizeof(float);
const uint16_t EEPROM_ROGER_TONE_ADDR = EEPROM_REPEATER_STATE_ADDR + sizeof(uint8_t);
const uint8_t EEPROM_REPEATER_ON = 0xA5;
const uint8_t EEPROM_REPEATER_OFF = 0x5A;

const uint8_t SR0_CMD_ERR = 0x01;
const uint8_t SR1_RDY     = 0x01;

struct IsdStatus {
  uint8_t sr0Lo;
  uint8_t sr0Hi;
  uint8_t sr1;
  uint16_t row;
};

struct Isd2Clip {
  uint16_t start;
  uint16_t end;
};

const Isd2Clip ISD2_CLIPS[] = {
  {0x000, 0x000},
  {0x010, 0x017},
  {0x018, 0x01E},
  {0x01F, 0x025},
  {0x026, 0x02C},
  {0x02D, 0x033},
  {0x034, 0x03B},
  {0x03C, 0x044},
  {0x045, 0x04A},
  {0x04B, 0x050},
  {0x051, 0x057},
  {0x058, 0x060},
  {0x061, 0x068},
  {0x069, 0x06F},
  {0x070, 0x077},
  {0x078, 0x07E},
  {0x07F, 0x087},
  {0x088, 0x092},
  {0x093, 0x09D},
  {0x09E, 0x0A7},
  {0x0A8, 0x0AE},
  {0x0C6, 0x0CC},
  {0x0CD, 0x0E0},
  {0x0E1, 0x0F0},
  {0x0F1, 0x0F9},
  {0x0FA, 0x106},
  {0x107, 0x111},
  {0x112, 0x11B},
  {0x11C, 0x123},
  {0x124, 0x12D},
  {0x12E, 0x137},
  {0x138, 0x141},
  {0x142, 0x14A},
  {0x14B, 0x156},
  {0x157, 0x161},
  {0x162, 0x168},
  {0x169, 0x173},
  {0x174, 0x17F},
  {0x180, 0x194}
};

const uint8_t ISD2_CLIP_DVADSAT = 20;
const uint8_t ISD2_CLIP_TRIDSAT = 21;
const uint8_t ISD2_CLIP_NAPATIE_AKUMULATORA = 22;
const uint8_t ISD2_CLIP_AKTUALNA_TEPLOTA = 23;
const uint8_t ISD2_CLIP_VOLTOV = 24;
const uint8_t ISD2_CLIP_STUPNOV_CELZIA = 25;
const uint8_t ISD2_CLIP_MINUS = 35;
const uint8_t ISD2_CLIP_ZIMA = 36;
const uint8_t ISD2_CLIP_KRASNY_DEN = 37;
const uint8_t ISD2_CLIP_PRELADUJE = 38;

unsigned long recordStartMs = 0;
unsigned long lastDurationMs = 0;
unsigned long sqlLowStartMs = 0;
unsigned long sqlHighStartMs = 0;
unsigned long lastDecodedDtmfToneMs = 0;
unsigned long lastRepeaterActivityMs = 0;
unsigned long nextAutoStatusReportMs = 0;

uint32_t autoStatusReportIntervalMs = 0;

float batteryConst = FALLBACK_BATTERY_CONST;
float lastBatteryVoltage = 0.0;
float lastTemperatureC = 0.0;

bool isRecording = false;
bool hasRecording = false;
bool sqlLowTiming = false;
bool sqlHighTiming = false;
bool lowBatteryRoger = false;
bool repeaterEnabled = true;
bool dtmfTonePrintPending = false;
uint8_t selectedRogerTone = ROGER_DEFAULT;
uint8_t decodedDtmfToneCount = 0;
char decodedDtmfTones[DTMF_TONE_LOG_MAX_LEN + 1];

OneWire oneWire(TEMP_PIN);
DallasTemperature ds18b20(&oneWire);

void loadBatteryConst();
void loadRepeaterState();
void loadRogerTone();
void updateMeasurements();
void playStartupRogerTones();
void initISD();
void printRepeaterState();
void printRogerToneState();
void updateDtmfTonePrint();
bool checkDtmfCommand(bool duringRecording);
void checkAutoStatusReport();
void setAutoStatusReportInterval(uint32_t intervalMs);
bool repeaterIdleForAutoStatus();
void markRepeaterActivity();
void checkSqlStart();
void checkSqlStop();
void startRecord();
void stopRecord(bool maxReached);
void repeaterPlaybackCycle();
bool readDtmfCode(char *code, bool duringRecording);
bool waitDtmfDigit(char *digit, bool duringRecording);
char decodeDtmfDigit();
void rememberDecodedDtmfTone(char digit);
void waitDtmfRelease(bool duringRecording);
bool handleChannelUpDtmfCommand(const char *code, bool duringRecording);
bool handleRogerDtmfCommand(const char *code, bool duringRecording);
void playEnableRogerConfirmation();
void playStatusReport();
void playAllRogerTones();
void playRogerChangeConfirmation();
void pulseChannelUp(uint8_t steps);
void saveRepeaterState(bool enabled);
void saveRogerTone(uint8_t rogerTone);
void abortRecordingToIdle();
void globalErase();
void playRogerToneByNumber(uint8_t rogerTone);
void playRogerTone();
float readBatteryVoltage();
void updateBatteryState();
float readTemperatureC();
void playLast();
void playIsd2SignedNumberOneDecimal(float value);
void playIsd2WholeNumber(uint8_t value);
void playIsd2Decimal(uint8_t decimalPart);
void playIsd2Clip(uint8_t clipId);
bool prepareISD2ForPlay();
void finishISD2Playback();
IsdStatus readStatusISD2();
bool waitReadyISD2(uint32_t timeoutMs);
void powerUpISD2();
void clrIntISD2();
void stopISD2();
void isdSelect();
void isdDeselect();
void isd2Select();
void isd2Deselect();
uint8_t xfer(uint8_t v);
IsdStatus readStatus();
bool waitReady(uint32_t timeoutMs);
void resetISD();
void powerUp();
void clrInt();
void stopISD();
void setupAPC_MIC_AUD();
bool sendSetCommand(uint8_t opcode, uint16_t start, uint16_t end);

void setup() {
  Serial.begin(9600);

  pinMode(ISD_SS, OUTPUT);
  digitalWrite(ISD_SS, HIGH);

  pinMode(ISD2_SS, OUTPUT);
  digitalWrite(ISD2_SS, HIGH);

  pinMode(PTT, OUTPUT);
  digitalWrite(PTT, LOW);

  pinMode(ROGER, OUTPUT);
  digitalWrite(ROGER, LOW);

  pinMode(CHANNEL_UP, OUTPUT);
  digitalWrite(CHANNEL_UP, LOW);

  pinMode(SQL, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);

  pinMode(DTMF_STD, INPUT);
  pinMode(DTMF_Q4, INPUT);
  pinMode(DTMF_Q3, INPUT);
  pinMode(DTMF_Q2, INPUT);
  pinMode(DTMF_Q1, INPUT);
  pinMode(TEMP_PIN, INPUT);

  ds18b20.begin();

  loadBatteryConst();
  loadRepeaterState();
  loadRogerTone();
  updateMeasurements();

  playStartupRogerTones();

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_SPEED, LSBFIRST, SPI_MODE3));

  initISD();

  lastRepeaterActivityMs = millis();

  Serial.println(F("CB OPAKOVAC PRIPRAVENY"));
  printRepeaterState();
  printRogerToneState();
}

void loop() {
  updateDtmfTonePrint();

  if (digitalRead(SQL) == LOW || isRecording) {
    markRepeaterActivity();
  }

  checkAutoStatusReport();

  if (!repeaterEnabled) {
    checkDtmfCommand(false);
    return;
  }

  if (!isRecording) {
    checkSqlStart();
    return;
  }

  if (millis() - recordStartMs >= MAX_RECORD_MS) {
    stopRecord(true);
    repeaterPlaybackCycle();
    return;
  }

  if (checkDtmfCommand(true)) {
    return;
  }

  if (isRecording) {
    checkSqlStop();
  }
}

void markRepeaterActivity() {
  lastRepeaterActivityMs = millis();
}

void setAutoStatusReportInterval(uint32_t intervalMs) {
  autoStatusReportIntervalMs = intervalMs;

  if (intervalMs == 0) {
    nextAutoStatusReportMs = 0;
    Serial.println(F("AUTO STATUS ZAKAZANY"));
    return;
  }

  nextAutoStatusReportMs = millis() + intervalMs;

  Serial.print(F("AUTO STATUS INTERVAL ms="));
  Serial.println(intervalMs);
}

bool repeaterIdleForAutoStatus() {
  if (!repeaterEnabled) {
    return false;
  }

  if (isRecording || hasRecording) {
    return false;
  }

  if (digitalRead(SQL) == LOW) {
    return false;
  }

  if (digitalRead(PTT) == HIGH) {
    return false;
  }

  return millis() - lastRepeaterActivityMs >= AUTO_STATUS_IDLE_MS;
}

void checkAutoStatusReport() {
  if (autoStatusReportIntervalMs == 0 || nextAutoStatusReportMs == 0) {
    return;
  }

  if ((int32_t)(millis() - nextAutoStatusReportMs) < 0) {
    return;
  }

  if (!repeaterIdleForAutoStatus()) {
    nextAutoStatusReportMs = millis() + AUTO_STATUS_DEFER_MS;
    Serial.println(F("AUTO STATUS ODLOZENY O 2 MIN"));
    return;
  }

  Serial.println(F("AUTO STATUS REPORT"));
  playStatusReport();
  markRepeaterActivity();
  nextAutoStatusReportMs = millis() + autoStatusReportIntervalMs;
}

void loadBatteryConst() {
  float storedConst;
  EEPROM.get(EEPROM_BATTERY_CONST_ADDR, storedConst);

  if (storedConst >= 0.001 && storedConst <= 0.1) {
    batteryConst = storedConst;
  } else {
    batteryConst = FALLBACK_BATTERY_CONST;
  }

  Serial.print(F("BAT CONST "));
  Serial.println(batteryConst, 6);
}

void loadRepeaterState() {
  uint8_t storedState = EEPROM.read(EEPROM_REPEATER_STATE_ADDR);

  if (storedState == EEPROM_REPEATER_OFF) {
    repeaterEnabled = false;
  } else {
    repeaterEnabled = true;

    if (storedState != EEPROM_REPEATER_ON) {
      EEPROM.update(EEPROM_REPEATER_STATE_ADDR, EEPROM_REPEATER_ON);
    }
  }
}

void saveRepeaterState(bool enabled) {
  repeaterEnabled = enabled;
  EEPROM.update(EEPROM_REPEATER_STATE_ADDR, enabled ? EEPROM_REPEATER_ON : EEPROM_REPEATER_OFF);
  printRepeaterState();
}

void loadRogerTone() {
  uint8_t storedRogerTone = EEPROM.read(EEPROM_ROGER_TONE_ADDR);

  if (storedRogerTone <= ROGER_COUNT) {
    selectedRogerTone = storedRogerTone;
  } else {
    selectedRogerTone = ROGER_DEFAULT;
    EEPROM.update(EEPROM_ROGER_TONE_ADDR, selectedRogerTone);
  }
}

void saveRogerTone(uint8_t rogerTone) {
  if (rogerTone > ROGER_COUNT) {
    return;
  }

  selectedRogerTone = rogerTone;
  EEPROM.update(EEPROM_ROGER_TONE_ADDR, selectedRogerTone);
  printRogerToneState();
}

void printRepeaterState() {
  Serial.print(F("OPAKOVAC "));
  Serial.println(repeaterEnabled ? F("ZAP") : F("VYP"));
}

void printRogerToneState() {
  Serial.print(F("ROGER "));

  if (selectedRogerTone == ROGER_OFF) {
    Serial.println(F("VYP"));
  } else {
    Serial.println(selectedRogerTone);
  }
}

float readBatteryVoltage() {
  uint32_t sum = 0;

  analogRead(BATTERY_PIN);
  delay(5);

  for (uint8_t i = 0; i < 16; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }

  float adc = sum / 16.0;
  return adc * batteryConst;
}

float readTemperatureC() {
  ds18b20.requestTemperatures();
  float temperatureC = ds18b20.getTempCByIndex(0);

  if (temperatureC == DEVICE_DISCONNECTED_C) {
    Serial.println(F("CHYBA: DS18B20"));
    return lastTemperatureC;
  }

  return temperatureC;
}

void updateMeasurements() {
  updateBatteryState();
  lastTemperatureC = readTemperatureC();

  Serial.print(F("TEMP "));
  Serial.print(lastTemperatureC, 1);
  Serial.println(F(" C"));
}

void updateBatteryState() {
  lastBatteryVoltage = readBatteryVoltage();
  lowBatteryRoger = lastBatteryVoltage < LOW_BATTERY_LIMIT;

  Serial.print(F("BAT "));
  Serial.print(lastBatteryVoltage, 1);
  Serial.println(F(" V"));
}

void checkSqlStart() {
  if (digitalRead(SQL) == LOW) {
    if (!sqlLowTiming) {
      sqlLowTiming = true;
      sqlLowStartMs = millis();
    }

    if (millis() - sqlLowStartMs >= SQL_START_STABLE_MS) {
      sqlLowTiming = false;
      startRecord();
    }
  } else {
    sqlLowTiming = false;
  }
}

void checkSqlStop() {
  if (digitalRead(SQL) == HIGH) {
    if (!sqlHighTiming) {
      sqlHighTiming = true;
      sqlHighStartMs = millis();
    }

    if (millis() - sqlHighStartMs >= SQL_STOP_STABLE_MS) {
      sqlHighTiming = false;
      stopRecord(false);
      repeaterPlaybackCycle();
    }
  } else {
    sqlHighTiming = false;
  }
}

bool checkDtmfCommand(bool duringRecording) {
  if (digitalRead(DTMF_STD) != HIGH) {
    return false;
  }

  char code[DTMF_CODE_LEN + 1];
  if (!readDtmfCode(code, duringRecording)) {
    return false;
  }

  if (strcmp(code, DTMF_CODE_ON) == 0) {
    Serial.println(F("DTMF ZAP PRIJATY"));
    saveRepeaterState(true);
    if (duringRecording) {
      abortRecordingToIdle();
    }
    playEnableRogerConfirmation();
    return true;
  }

  if (strcmp(code, DTMF_CODE_OFF) == 0) {
    Serial.println(F("DTMF VYP PRIJATY"));
    saveRepeaterState(false);
    if (duringRecording) {
      abortRecordingToIdle();
    }
    return true;
  }

  if (strcmp(code, DTMF_CODE_AUTO_STATUS_OFF) == 0) {
    Serial.println(F("DTMF AUTO STATUS VYP"));
    setAutoStatusReportInterval(0);
    return true;
  }

  if (strcmp(code, DTMF_CODE_AUTO_STATUS_1H) == 0) {
    Serial.println(F("DTMF AUTO STATUS 1H"));
    setAutoStatusReportInterval(3600000UL);
    return true;
  }

  if (strcmp(code, DTMF_CODE_AUTO_STATUS_4H) == 0) {
    Serial.println(F("DTMF AUTO STATUS 4H"));
    setAutoStatusReportInterval(14400000UL);
    return true;
  }

  if (strcmp(code, DTMF_CODE_AUTO_STATUS_12H) == 0) {
    Serial.println(F("DTMF AUTO STATUS 12H"));
    setAutoStatusReportInterval(43200000UL);
    return true;
  }

  if (strcmp(code, DTMF_CODE_AUTO_STATUS_24H) == 0) {
    Serial.println(F("DTMF AUTO STATUS 24H"));
    setAutoStatusReportInterval(86400000UL);
    return true;
  }

  if (strcmp(code, DTMF_CODE_STATUS_REPORT) == 0) {
    Serial.println(F("DTMF STATUS PRIJATY"));
    if (duringRecording) {
      abortRecordingToIdle();
    }
    playStatusReport();
    return true;
  }

  if (handleChannelUpDtmfCommand(code, duringRecording)) {
    return true;
  }

  if (strcmp(code, DTMF_CODE_ROGER_ALL) == 0) {
    Serial.println(F("DTMF VSETKY ROGER PRIJATE"));
    if (duringRecording) {
      abortRecordingToIdle();
    }
    playAllRogerTones();
    return true;
  }

  if (handleRogerDtmfCommand(code, duringRecording)) {
    return true;
  }

  Serial.print(F("DTMF NESPRAVNY KOD "));
  Serial.println(code);
  return false;
}

bool handleRogerDtmfCommand(const char *code, bool duringRecording) {
  uint8_t rogerTone;

  if (strcmp(code, DTMF_CODE_ROGER_OFF) == 0) {
    rogerTone = ROGER_OFF;
  } else if (strcmp(code, DTMF_CODE_ROGER_1) == 0) {
    rogerTone = 1;
  } else if (strcmp(code, DTMF_CODE_ROGER_2) == 0) {
    rogerTone = 2;
  } else if (strcmp(code, DTMF_CODE_ROGER_3) == 0) {
    rogerTone = 3;
  } else if (strcmp(code, DTMF_CODE_ROGER_4) == 0) {
    rogerTone = 4;
  } else if (strcmp(code, DTMF_CODE_ROGER_5) == 0) {
    rogerTone = 5;
  } else if (strcmp(code, DTMF_CODE_ROGER_6) == 0) {
    rogerTone = 6;
  } else if (strcmp(code, DTMF_CODE_ROGER_7) == 0) {
    rogerTone = 7;
  } else {
    return false;
  }

  Serial.println(F("DTMF ROGER PRIJATY"));
  saveRogerTone(rogerTone);

  if (duringRecording) {
    abortRecordingToIdle();
  }
  playRogerChangeConfirmation();

  return true;
}

bool handleChannelUpDtmfCommand(const char *code, bool duringRecording) {
  uint8_t steps;

  if (strcmp(code, DTMF_CODE_CHANNEL_UP_1) == 0) {
    steps = 1;
  } else if (strcmp(code, DTMF_CODE_CHANNEL_UP_2) == 0) {
    steps = 2;
  } else if (strcmp(code, DTMF_CODE_CHANNEL_UP_3) == 0) {
    steps = 3;
  } else if (strcmp(code, DTMF_CODE_CHANNEL_UP_4) == 0) {
    steps = 4;
  } else if (strcmp(code, DTMF_CODE_CHANNEL_UP_5) == 0) {
    steps = 5;
  } else {
    return false;
  }

  Serial.print(F("DTMF CHANNEL UP "));
  Serial.println(steps);

  if (duringRecording) {
    abortRecordingToIdle();
  }

  pulseChannelUp(steps);
  return true;
}

bool readDtmfCode(char *code, bool duringRecording) {
  uint8_t count = 0;

  while (count < DTMF_CODE_LEN) {
    char digit;
    if (!waitDtmfDigit(&digit, duringRecording)) {
      Serial.println(F("DTMF TIMEOUT - POKRACUJEM"));
      return false;
    }

    if (duringRecording && !isRecording) {
      return false;
    }

    code[count++] = digit;
  }

  code[DTMF_CODE_LEN] = '\0';

  char extraDigit;
  if (waitDtmfDigit(&extraDigit, duringRecording)) {
    Serial.println(F("DTMF DLHY KOD - POKRACUJEM"));
    return false;
  }

  if (duringRecording && !isRecording) {
    return false;
  }

  Serial.print(F("DTMF KOD "));
  Serial.println(code);
  return true;
}

bool waitDtmfDigit(char *digit, bool duringRecording) {
  uint32_t startMs = millis();

  while (millis() - startMs < DTMF_DIGIT_TIMEOUT_MS) {
    updateDtmfTonePrint();

    if (duringRecording && millis() - recordStartMs >= MAX_RECORD_MS) {
      stopRecord(true);
      repeaterPlaybackCycle();
      return false;
    }

    if (digitalRead(DTMF_STD) == HIGH) {
      delay(DTMF_STABLE_MS);

      if (digitalRead(DTMF_STD) == HIGH) {
        *digit = decodeDtmfDigit();
        rememberDecodedDtmfTone(*digit);
        waitDtmfRelease(duringRecording);
        return true;
      }
    }
  }

  return false;
}

void waitDtmfRelease(bool duringRecording) {
  while (digitalRead(DTMF_STD) == HIGH) {
    if (duringRecording && millis() - recordStartMs >= MAX_RECORD_MS) {
      stopRecord(true);
      repeaterPlaybackCycle();
      return;
    }

    delay(5);
  }
}

void rememberDecodedDtmfTone(char digit) {
  if (decodedDtmfToneCount >= DTMF_TONE_LOG_MAX_LEN) {
    decodedDtmfToneCount = 0;
  }

  decodedDtmfTones[decodedDtmfToneCount++] = digit;
  decodedDtmfTones[decodedDtmfToneCount] = '\0';
  lastDecodedDtmfToneMs = millis();
  dtmfTonePrintPending = true;
}

void updateDtmfTonePrint() {
  if (!dtmfTonePrintPending) {
    return;
  }

  if (digitalRead(DTMF_STD) == HIGH) {
    return;
  }

  if (millis() - lastDecodedDtmfToneMs < DTMF_PRINT_AFTER_LAST_MS) {
    return;
  }

  Serial.print(F("DTMF TONY "));
  Serial.println(decodedDtmfTones);
  decodedDtmfToneCount = 0;
  decodedDtmfTones[0] = '\0';
  dtmfTonePrintPending = false;
}

char decodeDtmfDigit() {
  uint8_t value = 0;

  if (digitalRead(DTMF_Q1) == HIGH) value |= 0x01;
  if (digitalRead(DTMF_Q2) == HIGH) value |= 0x02;
  if (digitalRead(DTMF_Q3) == HIGH) value |= 0x04;
  if (digitalRead(DTMF_Q4) == HIGH) value |= 0x08;

  switch (value) {
    case 0x01: return '1';
    case 0x02: return '2';
    case 0x03: return '3';
    case 0x04: return '4';
    case 0x05: return '5';
    case 0x06: return '6';
    case 0x07: return '7';
    case 0x08: return '8';
    case 0x09: return '9';
    case 0x0A: return '0';
    case 0x0B: return '*';
    case 0x0C: return '#';
    case 0x0D: return 'A';
    case 0x0E: return 'B';
    case 0x0F: return 'C';
    case 0x00: return 'D';
  }

  return '?';
}

void abortRecordingToIdle() {
  if (isRecording) {
    stopRecord(false);
  }

  hasRecording = false;
  lastDurationMs = 0;
  globalErase();
  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);
  delay(PAUSE_AFTER_PTT_OFF_MS);
  sqlLowTiming = false;
  sqlHighTiming = false;
}

void initISD() {
  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);

  resetISD();
  powerUp();
  stopISD();
  clrInt();
  waitReady(10000);
  setupAPC_MIC_AUD();
  stopISD();
  clrInt();
}

void repeaterPlaybackCycle() {
  markRepeaterActivity();

  if (!hasRecording || lastDurationMs < 200) {
    Serial.println(F("ZIADNA PLATNA NAHRAVKA"));
    globalErase();
    digitalWrite(PTT, LOW);
    digitalWrite(ROGER, LOW);
    delay(PAUSE_AFTER_PTT_OFF_MS);
    markRepeaterActivity();
    return;
  }

  delay(PAUSE_AFTER_RX_MS);

  digitalWrite(PTT, HIGH);
  delay(PAUSE_BEFORE_PLAY_MS);

  playLast();

  playRogerTone();
  updateMeasurements();

  globalErase();

  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);
  delay(PAUSE_AFTER_PTT_OFF_MS);

  hasRecording = false;
  lastDurationMs = 0;

  markRepeaterActivity();

  Serial.println(F("CAKAM NA SQL"));
}

void playStartupRogerTones() {
  digitalWrite(PTT, HIGH);
  delay(1000);

  if (selectedRogerTone == ROGER_OFF) {
    Serial.println(F("START ROGER VYP"));
  } else {
    Serial.print(F("START ROGER "));
    Serial.println(selectedRogerTone);
    playRogerToneByNumber(selectedRogerTone);
  }

  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);
  delay(PAUSE_AFTER_PTT_OFF_MS);
}

void playEnableRogerConfirmation() {
  digitalWrite(PTT, HIGH);
  delay(1000);

  if (selectedRogerTone == ROGER_OFF) {
    Serial.println(F("ZAP ROGER VYP"));
  } else {
    Serial.print(F("ZAP ROGER "));
    Serial.println(selectedRogerTone);
    playRogerToneByNumber(selectedRogerTone);
  }

  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);
  delay(PAUSE_AFTER_PTT_OFF_MS);
}

void playRogerChangeConfirmation() {
  digitalWrite(PTT, HIGH);
  delay(1000);

  if (selectedRogerTone == ROGER_OFF) {
    Serial.println(F("NASTAVENY ROGER VYP"));
  } else {
    Serial.print(F("NASTAVENY ROGER "));
    Serial.println(selectedRogerTone);
    playRogerToneByNumber(selectedRogerTone);
  }

  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);
  delay(PAUSE_AFTER_PTT_OFF_MS);
}

void playAllRogerTones() {
  digitalWrite(PTT, HIGH);
  delay(1000);

  for (uint8_t rogerTone = 1; rogerTone <= ROGER_COUNT; rogerTone++) {
    Serial.print(F("TEST ROGER "));
    Serial.println(rogerTone);
    playRogerToneByNumber(rogerTone);

    if (rogerTone < ROGER_COUNT) {
      delay(ROGER_ALL_PAUSE_MS);
    }
  }

  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);
  delay(PAUSE_AFTER_PTT_OFF_MS);
}

void pulseChannelUp(uint8_t steps) {
  for (uint8_t i = 0; i < steps; i++) {
    digitalWrite(CHANNEL_UP, HIGH);
    delay(CHANNEL_UP_PULSE_MS);
    digitalWrite(CHANNEL_UP, LOW);

    if (i + 1 < steps) {
      delay(CHANNEL_UP_PAUSE_MS);
    }
  }
}

void playStatusReport() {
  updateMeasurements();

  stopISD();
  clrInt();

  if (!prepareISD2ForPlay()) {
    return;
  }

  digitalWrite(PTT, HIGH);
  delay(1000);

  playIsd2Clip(ISD2_CLIP_NAPATIE_AKUMULATORA);
  playIsd2SignedNumberOneDecimal(lastBatteryVoltage);
  playIsd2Clip(ISD2_CLIP_VOLTOV);

  delay(STATUS_REPORT_PAUSE_MS);

  playIsd2Clip(ISD2_CLIP_AKTUALNA_TEPLOTA);
  playIsd2SignedNumberOneDecimal(lastTemperatureC);
  playIsd2Clip(ISD2_CLIP_STUPNOV_CELZIA);

  finishISD2Playback();
  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);
  delay(PAUSE_AFTER_PTT_OFF_MS);
}

void playIsd2SignedNumberOneDecimal(float value) {
  float absValue = value;

  if (value < 0.0) {
    playIsd2Clip(ISD2_CLIP_MINUS);
    absValue = -value;
  }

  uint16_t scaledValue = (uint16_t)(absValue * 10.0 + 0.5);
  uint8_t wholePart = scaledValue / 10;
  uint8_t decimalPart = scaledValue % 10;

  playIsd2WholeNumber(wholePart);
  playIsd2Decimal(decimalPart);
}

void playIsd2WholeNumber(uint8_t value) {
  if (value == 0) {
    return;
  }

  if (value <= 20) {
    playIsd2Clip(value);
    return;
  }

  if (value < 30) {
    playIsd2Clip(ISD2_CLIP_DVADSAT);
    playIsd2Clip(value - 20);
    return;
  }

  if (value == 30) {
    playIsd2Clip(ISD2_CLIP_TRIDSAT);
    return;
  }

  if (value < 40) {
    playIsd2Clip(ISD2_CLIP_TRIDSAT);
    playIsd2Clip(value - 30);
  }
}

void playIsd2Decimal(uint8_t decimalPart) {
  if (decimalPart >= 1 && decimalPart <= 9) {
    playIsd2Clip(25 + decimalPart);
  }
}

void playIsd2Clip(uint8_t clipId) {
  if (clipId == 0 || clipId >= (sizeof(ISD2_CLIPS) / sizeof(ISD2_CLIPS[0]))) {
    return;
  }

  Isd2Clip clip = ISD2_CLIPS[clipId];

  if (clip.end < clip.start) {
    return;
  }

  if (!prepareISD2ForPlay()) {
    return;
  }

  isd2Select();
  xfer(0x80);
  xfer(0x00);
  xfer(clip.start & 0xFF);
  xfer((clip.start >> 8) & 0x07);
  xfer(clip.end & 0xFF);
  xfer((clip.end >> 8) & 0x07);
  xfer(0x00);
  isd2Deselect();

  delay(((uint32_t)(clip.end - clip.start + 1) * ISD2_ROW_TIME_MS) + ISD2_CLIP_PAUSE_MS);

  finishISD2Playback();
}

bool prepareISD2ForPlay() {
  digitalWrite(ISD_SS, HIGH);
  digitalWrite(ISD2_SS, HIGH);
  stopISD2();
  clrIntISD2();
  powerUpISD2();
  stopISD2();
  clrIntISD2();
  return waitReadyISD2(ISD2_READY_TIMEOUT_MS);
}

void finishISD2Playback() {
  stopISD2();
  clrIntISD2();
  waitReadyISD2(ISD2_READY_TIMEOUT_MS);

  digitalWrite(ISD2_SS, HIGH);
  digitalWrite(ISD_SS, HIGH);
}

IsdStatus readStatusISD2() {
  IsdStatus s;

  isd2Select();
  s.sr0Lo = xfer(0x05);
  s.sr0Hi = xfer(0x00);
  s.sr1   = xfer(0x00);
  isd2Deselect();

  s.row = ((uint16_t)s.sr0Hi << 3) | (s.sr0Lo >> 5);

  return s;
}

bool waitReadyISD2(uint32_t timeoutMs) {
  uint32_t t0 = millis();

  while (true) {
    IsdStatus s = readStatusISD2();

    if (s.sr1 & SR1_RDY) {
      return true;
    }

    if (millis() - t0 > timeoutMs) {
      Serial.println(F("CHYBA: ISD2 NIE JE READY"));
      return false;
    }

    delay(20);
  }
}

void powerUpISD2() {
  isd2Select();
  xfer(0x01);
  xfer(0x00);
  isd2Deselect();
  delay(300);
}

void clrIntISD2() {
  isd2Select();
  xfer(0x04);
  xfer(0x00);
  isd2Deselect();
  delay(80);
}

void stopISD2() {
  isd2Select();
  xfer(0x02);
  xfer(0x00);
  isd2Deselect();
  delay(250);
}

void playRogerTone() {
  if (selectedRogerTone == ROGER_OFF) {
    Serial.println(F("ROGER VYP"));
    digitalWrite(ROGER, LOW);
    return;
  }

  if (lowBatteryRoger) {
    Serial.println(F("LOW BAT ROGER"));

    tone(ROGER, 400, 500);
    delay(550);
    noTone(ROGER);
    digitalWrite(ROGER, LOW);
    return;
  }

  Serial.print(F("ROGER "));
  Serial.println(selectedRogerTone);
  playRogerToneByNumber(selectedRogerTone);
}

void playRogerToneByNumber(uint8_t rogerTone) {
  switch (rogerTone) {
    case 1:
      tone(ROGER, 1200, 80);
      delay(110);
      tone(ROGER, 1650, 80);
      delay(110);
      tone(ROGER, 2200, 120);
      delay(150);
      break;

    case 2:
      tone(ROGER, 1000, 70);
      delay(100);
      tone(ROGER, 1500, 70);
      delay(100);
      tone(ROGER, 1000, 120);
      delay(150);
      break;

    case 3:
      tone(ROGER, 1800, 90);
      delay(120);
      tone(ROGER, 1200, 90);
      delay(120);
      tone(ROGER, 800, 140);
      delay(170);
      break;

    case 4:
      tone(ROGER, 2200, 60);
      delay(90);
      tone(ROGER, 1800, 60);
      delay(90);
      tone(ROGER, 1400, 60);
      delay(90);
      tone(ROGER, 1000, 140);
      delay(170);
      break;

    case 5:
      tone(ROGER, 700, 100);
      delay(130);
      tone(ROGER, 900, 100);
      delay(130);
      tone(ROGER, 1100, 100);
      delay(130);
      tone(ROGER, 1300, 180);
      delay(210);
      break;

    case 6:
      tone(ROGER, 1375, 100);
      delay(90);
      break;

    case 7:
      tone(ROGER, 1375, 100);
      delay(85);
      noTone(ROGER);
      delay(35);
      tone(ROGER, 1375, 100);
      delay(85);
      break;
  }

  noTone(ROGER);
  digitalWrite(ROGER, LOW);
}

void isdSelect() {
  digitalWrite(ISD2_SS, HIGH);
  digitalWrite(ISD_SS, HIGH);
  delayMicroseconds(100);
  digitalWrite(ISD_SS, LOW);
  delayMicroseconds(100);
}

void isdDeselect() {
  delayMicroseconds(100);
  digitalWrite(ISD_SS, HIGH);
  delayMicroseconds(100);
}

void isd2Select() {
  digitalWrite(ISD_SS, HIGH);
  digitalWrite(ISD2_SS, HIGH);
  delayMicroseconds(100);
  digitalWrite(ISD2_SS, LOW);
  delayMicroseconds(100);
}

void isd2Deselect() {
  delayMicroseconds(100);
  digitalWrite(ISD2_SS, HIGH);
  delayMicroseconds(100);
}

uint8_t xfer(uint8_t v) {
  delayMicroseconds(50);
  uint8_t r = SPI.transfer(v);
  delayMicroseconds(50);
  return r;
}

IsdStatus readStatus() {
  IsdStatus s;

  isdSelect();
  s.sr0Lo = xfer(0x05);
  s.sr0Hi = xfer(0x00);
  s.sr1   = xfer(0x00);
  isdDeselect();

  s.row = ((uint16_t)s.sr0Hi << 3) | (s.sr0Lo >> 5);

  return s;
}

bool waitReady(uint32_t timeoutMs) {
  uint32_t t0 = millis();

  while (true) {
    IsdStatus s = readStatus();

    if (s.sr1 & SR1_RDY) {
      return true;
    }

    if (millis() - t0 > timeoutMs) {
      Serial.println(F("CHYBA: ISD NIE JE READY"));
      return false;
    }

    delay(20);
  }
}

void resetISD() {
  isdSelect();
  xfer(0x03);
  xfer(0x00);
  isdDeselect();

  delay(500);
}

void powerUp() {
  isdSelect();
  xfer(0x01);
  xfer(0x00);
  isdDeselect();

  delay(300);
  clrInt();
  waitReady(10000);
}

void clrInt() {
  isdSelect();
  xfer(0x04);
  xfer(0x00);
  isdDeselect();

  delay(80);
}

void stopISD() {
  isdSelect();
  xfer(0x02);
  xfer(0x00);
  isdDeselect();

  delay(250);
}

void setupAPC_MIC_AUD() {
  isdSelect();
  xfer(0x65);
  xfer(0x00);
  xfer(0x04);
  isdDeselect();

  delay(300);
  waitReady(10000);
  clrInt();
}

bool sendSetCommand(uint8_t opcode, uint16_t start, uint16_t end) {
  if (end < start || end > MAX_END_ADDR) {
    Serial.println(F("CHYBA: ZLY ROZSAH"));
    return false;
  }

  clrInt();

  if (!waitReady(10000)) {
    return false;
  }

  isdSelect();
  xfer(opcode);
  xfer(0x00);
  xfer(start & 0xFF);
  xfer((start >> 8) & 0x07);
  xfer(end & 0xFF);
  xfer((end >> 8) & 0x07);
  xfer(0x00);
  isdDeselect();

  delay(120);

  IsdStatus s = readStatus();

  if (s.sr0Lo & SR0_CMD_ERR) {
    Serial.println(F("CHYBA: ISD ODMETOL PRIKAZ"));
    clrInt();
    return false;
  }

  return true;
}

void startRecord() {
  if (isRecording || !repeaterEnabled) return;

  markRepeaterActivity();

  digitalWrite(PTT, LOW);
  digitalWrite(ROGER, LOW);

  stopISD();
  clrInt();

  if (!sendSetCommand(0x81, RECORD_START_ADDR, MAX_END_ADDR)) {
    return;
  }

  recordStartMs = millis();
  lastDurationMs = 0;
  hasRecording = false;
  isRecording = true;
  sqlHighTiming = false;

  Serial.println(F("REC START"));
}

void stopRecord(bool maxReached) {
  if (!isRecording) return;

  markRepeaterActivity();

  stopISD();

  lastDurationMs = millis() - recordStartMs;

  if (lastDurationMs > MAX_RECORD_MS) {
    lastDurationMs = MAX_RECORD_MS;
  }

  isRecording = false;
  hasRecording = lastDurationMs > 200;
  sqlLowTiming = false;
  sqlHighTiming = false;

  delay(300);
  waitReady(10000);
  clrInt();

  if (maxReached) {
    Serial.println(F("MAX CAS REC DOSIAHNUTY"));
  }

  Serial.print(F("REC STOP ms="));
  Serial.println(lastDurationMs);
}

void playLast() {
  if (!hasRecording || lastDurationMs < 200) {
    Serial.println(F("NIE JE NAHRAVKA"));
    return;
  }

  stopISD();
  clrInt();

  Serial.println(F("PLAY"));

  if (!sendSetCommand(0x80, RECORD_START_ADDR, MAX_END_ADDR)) {
    return;
  }

  if (lastDurationMs > BATTERY_MEASURE_BEFORE_END_MS) {
    delay(lastDurationMs - BATTERY_MEASURE_BEFORE_END_MS);
    updateBatteryState();
    delay(BATTERY_MEASURE_BEFORE_END_MS);
  } else {
    updateBatteryState();
    delay(lastDurationMs);
  }

  stopISD();
  clrInt();

  Serial.println(F("PLAY KONIEC"));
}

void globalErase() {
  Serial.println(F("GE"));

  stopISD();
  clrInt();
  waitReady(10000);

  isdSelect();
  xfer(0x43);
  xfer(0x00);
  isdDeselect();

  delay(GLOBAL_ERASE_WAIT_MS);

  clrInt();

  Serial.println(F("GE HOTOVO"));
}
