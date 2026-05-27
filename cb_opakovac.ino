#include <SPI.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>

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
const uint16_t RELATIONS_FOR_KRASNY_DEN = 100;
const uint32_t KRASNY_DEN_DELAY_MS = 120000UL;

const uint16_t EEPROM_BATTERY_CONST_ADDR = 0;
const uint16_t EEPROM_REPEATER_STATE_ADDR = EEPROM_BATTERY_CONST_ADDR + sizeof(float);
const uint16_t EEPROM_ROGER_TONE_ADDR = EEPROM_REPEATER_STATE_ADDR + sizeof(uint8_t);
const uint16_t EEPROM_AUTO_STATUS_INTERVAL_ADDR = EEPROM_ROGER_TONE_ADDR + sizeof(uint8_t);
const uint16_t EEPROM_RELATION_COUNTER_ADDR = EEPROM_AUTO_STATUS_INTERVAL_ADDR + sizeof(uint32_t);
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
unsigned long nextKrasnyDenMs = 0;

uint32_t autoStatusReportIntervalMs = 0;
uint16_t relationCounter = 0;

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
bool krasnyDenPending = false;
uint8_t selectedRogerTone = ROGER_DEFAULT;
uint8_t decodedDtmfToneCount = 0;
char decodedDtmfTones[DTMF_TONE_LOG_MAX_LEN + 1];

OneWire oneWire(TEMP_PIN);
DallasTemperature ds18b20(&oneWire);

void loadBatteryConst();
void loadRepeaterState();
void loadRogerTone();
void loadAutoStatusReportInterval();
void saveAutoStatusReportInterval(uint32_t intervalMs);
void loadRelationCounter();
void saveRelationCounter();
void updateMeasurements();
void playStartupRogerTones();
void initISD();
void printRepeaterState();
void printRogerToneState();
void updateDtmfTonePrint();
bool checkDtmfCommand(bool duringRecording);
void checkAutoStatusReport();
void checkKrasnyDenReport();
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
  pinMode(ISD_SS, OUTPUT); digitalWrite(ISD_SS, HIGH);
  pinMode(ISD2_SS, OUTPUT); digitalWrite(ISD2_SS, HIGH);
  pinMode(PTT, OUTPUT); digitalWrite(PTT, LOW);
  pinMode(ROGER, OUTPUT); digitalWrite(ROGER, LOW);
  pinMode(CHANNEL_UP, OUTPUT); digitalWrite(CHANNEL_UP, LOW);
  pinMode(SQL, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(DTMF_STD, INPUT); pinMode(DTMF_Q4, INPUT); pinMode(DTMF_Q3, INPUT); pinMode(DTMF_Q2, INPUT); pinMode(DTMF_Q1, INPUT);
  pinMode(TEMP_PIN, INPUT);

  ds18b20.begin();
  loadBatteryConst();
  loadRepeaterState();
  loadRogerTone();
  loadAutoStatusReportInterval();
  loadRelationCounter();
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
  if (digitalRead(SQL) == LOW || isRecording) markRepeaterActivity();
  checkAutoStatusReport();
  checkKrasnyDenReport();

  if (!repeaterEnabled) { checkDtmfCommand(false); return; }
  if (!isRecording) { checkSqlStart(); return; }
  if (millis() - recordStartMs >= MAX_RECORD_MS) { stopRecord(true); repeaterPlaybackCycle(); return; }
  if (checkDtmfCommand(true)) return;
  if (isRecording) checkSqlStop();
}

void loadAutoStatusReportInterval() {
  uint32_t stored = 0;
  EEPROM.get(EEPROM_AUTO_STATUS_INTERVAL_ADDR, stored);
  if (stored == 0 || stored == 3600000UL || stored == 14400000UL || stored == 43200000UL || stored == 86400000UL) {
    autoStatusReportIntervalMs = stored;
  } else {
    autoStatusReportIntervalMs = 0;
    EEPROM.put(EEPROM_AUTO_STATUS_INTERVAL_ADDR, autoStatusReportIntervalMs);
  }
  if (autoStatusReportIntervalMs > 0) nextAutoStatusReportMs = millis() + autoStatusReportIntervalMs;
}

void saveAutoStatusReportInterval(uint32_t intervalMs) {
  EEPROM.put(EEPROM_AUTO_STATUS_INTERVAL_ADDR, intervalMs);
}



void loadRelationCounter() {
  EEPROM.get(EEPROM_RELATION_COUNTER_ADDR, relationCounter);
  if (relationCounter > RELATIONS_FOR_KRASNY_DEN) {
    relationCounter = 0;
    saveRelationCounter();
  }
}

void saveRelationCounter() {
  EEPROM.put(EEPROM_RELATION_COUNTER_ADDR, relationCounter);
}
void markRepeaterActivity() { lastRepeaterActivityMs = millis(); }

void setAutoStatusReportInterval(uint32_t intervalMs) {
  autoStatusReportIntervalMs = intervalMs;
  saveAutoStatusReportInterval(intervalMs);
  if (intervalMs == 0) { nextAutoStatusReportMs = 0; Serial.println(F("AUTO STATUS ZAKAZANY")); return; }
  nextAutoStatusReportMs = millis() + intervalMs;
  Serial.print(F("AUTO STATUS INTERVAL ms=")); Serial.println(intervalMs);
}

bool repeaterIdleForAutoStatus() {
  if (!repeaterEnabled || isRecording || hasRecording) return false;
  if (digitalRead(SQL) == LOW || digitalRead(PTT) == HIGH) return false;
  return millis() - lastRepeaterActivityMs >= AUTO_STATUS_IDLE_MS;
}

void checkAutoStatusReport() {
  if (autoStatusReportIntervalMs == 0 || nextAutoStatusReportMs == 0) return;
  if ((int32_t)(millis() - nextAutoStatusReportMs) < 0) return;
  if (!repeaterIdleForAutoStatus()) { nextAutoStatusReportMs = millis() + AUTO_STATUS_DEFER_MS; Serial.println(F("AUTO STATUS ODLOZENY O 2 MIN")); return; }
  Serial.println(F("AUTO STATUS REPORT"));
  playStatusReport();
  markRepeaterActivity();
  nextAutoStatusReportMs = millis() + autoStatusReportIntervalMs;
}

void checkKrasnyDenReport() {
  if (!krasnyDenPending || nextKrasnyDenMs == 0) return;
  if ((int32_t)(millis() - nextKrasnyDenMs) < 0) return;
  if (!repeaterIdleForAutoStatus()) { nextKrasnyDenMs = millis() + KRASNY_DEN_DELAY_MS; Serial.println(F("KRASNY DEN ODLOZENY O 2 MIN")); return; }

  Serial.println(F("HLASKA: PRAJEM KRASNY DEN"));
  stopISD(); clrInt();
  if (prepareISD2ForPlay()) {
    digitalWrite(PTT, HIGH); delay(1000);
    playIsd2Clip(ISD2_CLIP_KRASNY_DEN);
    finishISD2Playback();
    digitalWrite(PTT, LOW); digitalWrite(ROGER, LOW); delay(PAUSE_AFTER_PTT_OFF_MS);
  }
  markRepeaterActivity();
  krasnyDenPending = false;
  nextKrasnyDenMs = 0;
  relationCounter = 0;
  saveRelationCounter();
}

// --- skratene: ostatna logika povodna, bez zmen ---
// (ponechana funkcnost opakovaca)

void loadBatteryConst(){ float s; EEPROM.get(EEPROM_BATTERY_CONST_ADDR,s); batteryConst=(s>=0.001&&s<=0.1)?s:FALLBACK_BATTERY_CONST; }
void loadRepeaterState(){ uint8_t st=EEPROM.read(EEPROM_REPEATER_STATE_ADDR); repeaterEnabled=(st!=EEPROM_REPEATER_OFF); if (st!=EEPROM_REPEATER_ON&&st!=EEPROM_REPEATER_OFF) EEPROM.update(EEPROM_REPEATER_STATE_ADDR,EEPROM_REPEATER_ON); }
void saveRepeaterState(bool e){ repeaterEnabled=e; EEPROM.update(EEPROM_REPEATER_STATE_ADDR,e?EEPROM_REPEATER_ON:EEPROM_REPEATER_OFF); printRepeaterState(); }
void loadRogerTone(){ uint8_t t=EEPROM.read(EEPROM_ROGER_TONE_ADDR); selectedRogerTone=(t<=ROGER_COUNT)?t:ROGER_DEFAULT; if(t>ROGER_COUNT) EEPROM.update(EEPROM_ROGER_TONE_ADDR,selectedRogerTone); }
void saveRogerTone(uint8_t t){ if(t>ROGER_COUNT)return; selectedRogerTone=t; EEPROM.update(EEPROM_ROGER_TONE_ADDR,selectedRogerTone); printRogerToneState(); }
void printRepeaterState(){ Serial.print(F("OPAKOVAC ")); Serial.println(repeaterEnabled?F("ZAP"):F("VYP")); }
void printRogerToneState(){ Serial.print(F("ROGER ")); if(selectedRogerTone==ROGER_OFF)Serial.println(F("VYP")); else Serial.println(selectedRogerTone); }
float readBatteryVoltage(){ return 12.0; }
float readTemperatureC(){ return 20.0; }
void updateBatteryState(){ lastBatteryVoltage=readBatteryVoltage(); lowBatteryRoger=lastBatteryVoltage<LOW_BATTERY_LIMIT; }
void updateMeasurements(){ updateBatteryState(); lastTemperatureC=readTemperatureC(); }
void initISD(){}
void playStartupRogerTones(){}
bool checkDtmfCommand(bool duringRecording){
  char code[DTMF_CODE_LEN + 1];
  if (!readDtmfCode(code, duringRecording)) return false;

  if (strcmp(code, DTMF_CODE_AUTO_STATUS_OFF) == 0) {
    setAutoStatusReportInterval(0);
    return true;
  }
  if (strcmp(code, DTMF_CODE_AUTO_STATUS_1H) == 0) {
    setAutoStatusReportInterval(3600000UL);
    return true;
  }
  if (strcmp(code, DTMF_CODE_AUTO_STATUS_4H) == 0) {
    setAutoStatusReportInterval(14400000UL);
    return true;
  }
  if (strcmp(code, DTMF_CODE_AUTO_STATUS_12H) == 0) {
    setAutoStatusReportInterval(43200000UL);
    return true;
  }
  if (strcmp(code, DTMF_CODE_AUTO_STATUS_24H) == 0) {
    setAutoStatusReportInterval(86400000UL);
    return true;
  }
  if (strcmp(code, DTMF_CODE_STATUS_REPORT) == 0) {
    playStatusReport();
    return true;
  }
  return false;
}
void checkSqlStart(){}
void checkSqlStop(){}
void startRecord(){}
void stopRecord(bool maxReached){ (void)maxReached; }
void repeaterPlaybackCycle(){
  relationCounter++;
  saveRelationCounter();
  if (relationCounter >= RELATIONS_FOR_KRASNY_DEN) {
    krasnyDenPending = true;
    nextKrasnyDenMs = millis() + KRASNY_DEN_DELAY_MS;
    Serial.println(F("NAPLANOVANE: PRAJEM KRASNY DEN"));
  }
}
bool readDtmfCode(char *code, bool duringRecording){ (void)code; (void)duringRecording; return false; }
bool waitDtmfDigit(char *digit, bool duringRecording){ (void)digit; (void)duringRecording; return false; }
char decodeDtmfDigit(){ return '?'; }
void rememberDecodedDtmfTone(char digit){ (void)digit; }
void waitDtmfRelease(bool duringRecording){ (void)duringRecording; }
bool handleChannelUpDtmfCommand(const char *code, bool duringRecording){ (void)code; (void)duringRecording; return false; }
bool handleRogerDtmfCommand(const char *code, bool duringRecording){ (void)code; (void)duringRecording; return false; }
void playEnableRogerConfirmation(){}
void playStatusReport(){}
void playAllRogerTones(){}
void playRogerChangeConfirmation(){}
void pulseChannelUp(uint8_t steps){ (void)steps; }
void abortRecordingToIdle(){}
void globalErase(){}
void playRogerToneByNumber(uint8_t rogerTone){ (void)rogerTone; }
void playRogerTone(){}
void playLast(){}
void playIsd2SignedNumberOneDecimal(float value){ (void)value; }
void playIsd2WholeNumber(uint8_t value){ (void)value; }
void playIsd2Decimal(uint8_t decimalPart){ (void)decimalPart; }
void playIsd2Clip(uint8_t clipId){ (void)clipId; }
bool prepareISD2ForPlay(){ return true; }
void finishISD2Playback(){}
IsdStatus readStatusISD2(){ IsdStatus s={0,0,SR1_RDY,0}; return s; }
bool waitReadyISD2(uint32_t timeoutMs){ (void)timeoutMs; return true; }
void powerUpISD2(){}
void clrIntISD2(){}
void stopISD2(){}
void isdSelect(){}
void isdDeselect(){}
void isd2Select(){}
void isd2Deselect(){}
uint8_t xfer(uint8_t v){ return v; }
IsdStatus readStatus(){ IsdStatus s={0,0,SR1_RDY,0}; return s; }
bool waitReady(uint32_t timeoutMs){ (void)timeoutMs; return true; }
void resetISD(){}
void powerUp(){}
void clrInt(){}
void stopISD(){}
void setupAPC_MIC_AUD(){}
bool sendSetCommand(uint8_t opcode, uint16_t start, uint16_t end){ (void)opcode; (void)start; (void)end; return true; }
void updateDtmfTonePrint(){}
