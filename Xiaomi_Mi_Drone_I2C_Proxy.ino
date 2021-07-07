// Xiaomi Mi Drone I2C Proxy by rjlexx
// Powered by 4pda.ru community
// This application allows to use Arduino
// like a I2C Proxy between Xiaomi Mi Drone and locked battery.
// It's strongly recommended to change bad(undervoltaged) bank
// in locked battery to good bank from another locked battery
// before using I2C proxy for unlock battery!
// The main functionality is changing response from
// locked battery to drone for 2 commands:
// 0xFF 0x03 0x4A 0x00 0x02 - undervoltage times - always send zero value - 0x0 0x0 0x0 0x0
// 0x07 - battery status - always send good status - 0x04 0x0 0x0B 0x0B
// Also this app allows to send commands from Serial to battery
// for analyzing purpose. It's better to set DEBUG_MODE=0 when
// analyzing is not needed

// Connection:
// I2C Bus from drone should be connected to the hardware I2C port,
// bacause only hardware I2C supports slave mode.
// For the Arduino Pro Mini it's A4(SDA) and A5(SCL) pins.
// I2C Bus from battery could be connected to any PWM pins.
// These pins should be defined as I2C_BATTERY_SDA and I2C_BATTERY_SCL
// Using pin 5 (SDA) and 6 (SCL) in this example.
// SCL Green
// SDA WGreen

//#include "Wire.h"
#include <SoftwareWire.h>

// DEBUG_MODE
// 1 - enabled. Prints all communication by I2C to Serial and handles commands from Serial
// 0 - disabled.
#define DEBUG_MODE        0
#define BATTERY_ADDRESS   0x0B
#define I2C_BATTERY_SDA   5
#define I2C_BATTERY_SCL   6
SoftwareWire WireBat(I2C_BATTERY_SDA, I2C_BATTERY_SCL, false, true);

//Standart commands - commands with on byte length
// UNKNOWN COMMAND. BatteryMode? value=0x00 when Power On, 0x04 then 0x06 when Shutting Down
#define Cmd_06                      0x06
// Battery Status value=0x04 - good, 0x84 - locked
#define Cmd_BatteryStatus           0x07
// Temprature, Kelvins
#define Cmd_Temperature             0x08
// Common voltage, mV
#define Cmd_Voltage                 0x09
// Current, mA. Negative for discharge, positive for charge
#define Cmd_Current                 0x0A
// AverageCurrent, mA. Negative for discharge, positive for charge
#define Cmd_AverageCurrent          0x0B
// Remaing Capacity,mAH
#define Cmd_RemainingCapacity       0x0F
// Device Name
#define Cmd_DeviceName              0x16
// RelativeStateOfCharge in percent
#define Cmd_RelativeStateOfCharge   0x19
// Battery Discharge\Charge Cycles Count
#define Cmd_Cycles                  0x1B
// Total Capacity,mAH
#define Cmd_Capacity                0x1D
// UNKNOWN COMMAND. DeviceChemistry? value=0x13+0xEC=5100
#define Cmd_1F         0x1F
// UNKNOWN COMMAND. Changes after write to FF 60 and FF 61. Authentication?
#define Cmd_33                      0x33
// UNKNOWN COMMAND. Changes after write to FF 60 and FF 61. Authentication?
#define Cmd_34                      0x34
// Bank1 voltage, mV
#define Cmd_VoltageBank1            0x3C
// Bank2 voltage, mV
#define Cmd_VoltageBank2            0x3D
// Bank3 voltage, mV
#define Cmd_VoltageBank3            0x3E
// Bank4 voltage, mV
#define Cmd_VoltageBank4            0x3F

// Custom commands - commands with 5 bytes length with following format:
// 1. 0xFF - static
// 2. 0x03 - static
// 3. command byte, like 0x4A
// 4. 0x00 - static
// 5. response length byte
// Example: 0xFF 0x03 0x4A 0x00 0x02
// Undervoltage Count
#define Cmd_UndervoltageCount       0x4A
// UNKNOWN COMMAND. Appears ones per session. value=0x05
#define Cmd_76                      0x76
// UNKNOWN COMMAND. value=0x01
#define Cmd_C4                      0xC4
// UNKNOWN COMMAND. value=0x01
#define Cmd_C5                      0xC5

bool batteryLoaded = false;
volatile byte commandFirstByte;
volatile byte commandSecondByte;
volatile byte commandThirdByte;
volatile int commandLength;
const int crcBufLen = 64;
volatile byte buf[crcBufLen];
volatile int crcBufPos = 0;
const int requestBufLen = 32;
volatile byte requestBuf[requestBufLen];
const int responseBufLen = 32;
volatile byte responseBuf[responseBufLen];
bool requestRecieved = false;
bool responseSent = false;
volatile int responseLen;

void setup() {
  WireBat.setClock(140000UL);
  WireBat.setTimeout(5);
  WireBat.begin();
  Serial.begin(115200);
  if (DEBUG_MODE) {
    Serial.println(F("Battery Proxy Loaded"));
  }
}

void loop() {
  // Initialize I2C for drone in slave mode after battery's I2C initialized
  while (!batteryLoaded) {
    if (deviceAvailable(BATTERY_ADDRESS)) {
      batteryLoaded = true;
      if (DEBUG_MODE) {
        Serial.println(F("Battery's SMBus initialized."));
        Serial.println(F("Initializing drone's SMBus."));
      }
      Wire.begin(BATTERY_ADDRESS);
      Wire.onReceive(receiveEvent);
      Wire.onRequest(requestEvent);
    }
  }

  if (DEBUG_MODE) {
    // Prints recieved request from drone to Serial
    if (requestRecieved) {
      Serial.println(F("Recieved request from drone"));
      for (int i = 0; i < commandLength; i++) {
        Serial.println(byteToStrHex(requestBuf[i]));
      }
    }

    // Prints recieved response from battery to Serial
    if (responseSent) {
      Serial.print(F("Response from battery with len="));
      Serial.println(responseLen);
      for (int i = 0; i < responseLen; i++) {
        Serial.println(byteToStrHex(responseBuf[i]));
      }
      responseSent = false;
    }
  }

  handleCommandsFromSerial();

  delay(10);
}

// Handles commands from drone and sends it to battery
void receiveEvent(int len) {
  requestRecieved = false;
  commandLength = len;
  commandFirstByte = 0;
  commandSecondByte = 0;
  commandThirdByte = 0;
  if (DEBUG_MODE) {
    for (int i = 0; i < crcBufLen; i++) buf[i] = 0;
    for (int i = 0; i < requestBufLen; i++) requestBuf[i] = 0;
    crcBufPos = 0;
  }
  beginTransmission(BATTERY_ADDRESS);
  for (int i = 0; i < len; i++) {
    byte b = Wire.read();
    writeByte(b);
    if (DEBUG_MODE) {
      buf[crcBufPos++] = b;
      requestBuf[i] = b;
    }
    if (i == 0) commandFirstByte = b;
    if (i == 1) commandSecondByte = b;
    if (i == 2) commandThirdByte = b;
  }

  //Commands FF 60, FF 61 and FF 62 doesn't ask for response and we need to send stop
  if (commandFirstByte == 0xFF && (commandSecondByte == 0x60 || commandSecondByte == 0x61 || commandSecondByte == 0x62))
    endTransmission(true);
  else
    endTransmission(false);

  requestRecieved = true;
}

// Handles requests for response from drone
// then requests the response from battery
// and sends response to drone with changes for some commands:
// 0xFF 0x03 0x4A 0x00 0x02 - undervoltage times - always send 0x0 0x0 0x0 0x0
// 0x07 - battery status - always send good status 0x04 0x0 0x0b 0x0b
void requestEvent() {
  responseSent = false;
  responseLen = getResponseLen(commandFirstByte, commandThirdByte);
  if (DEBUG_MODE)
    for (int i = 0; i < responseBufLen; i++) responseBuf[i] = 0;
  int count = 0;
  int status = requestFrom(BATTERY_ADDRESS);
  while (count < responseLen)
  {
    byte b = readByte(count == responseLen - 1);
    if (commandThirdByte == Cmd_UndervoltageCount) {
      b = 0x0;
    }
    else if (commandFirstByte == 0x07) {
      switch (count) {
        case 0:
          b = 0x04;
          break;
        case 1:
          b = 0x0;
          break;
        case 2:
          b = 0x0b;
          break;
        case 3:
          b = 0x0b;
          break;
        default:
          break;
      }
    }

    // Here is using custom low level write w\o buffer (Wire.llWrite).
    // But last byte should be written throught standart Wire.write function,
    // otherwise Wire lib will write 0x00 byte.
    if (count == responseLen - 1)
      Wire.write(b);
    else {
      Wire.llWrite(b, false);
    }
    if (DEBUG_MODE) {
      responseBuf[count] = b;
      buf[crcBufPos++] = b;
    }
    count++;
  }
  endTransmission(true);
  responseSent = true;
}

// Returns count of bytes that should be requested from battery
// and returned to drone for each command.
// It's matter because communication with drone fails
// if send him more or less bytes than drone expects.
int getResponseLen(byte firstByte, byte thirdByte) {
  switch (firstByte) {
    case 0x06:
      return 4;
    case 0x07:
      return 4;
    case 0x08:
      return 4;
    case 0x09:
      return 4;
    case 0x0a:
      return 4;
    case 0x0b:
      return 4;
    case 0x0f:
      return 4;
    case 0x19:
      return 4;
    case 0x1b:
      return 4;
    case 0x1d:
      return 4;
    case 0x1f:
      return 4;
    case 0x33:
      return 2;
    case 0x34:
      return 2;
    case 0x3c:
      return 4;
    case 0x3d:
      return 4;
    case 0x3e:
      return 4;
    case 0x3f:
      return 4;
    case 0xff:
      switch (thirdByte) {
        case 0x4a:
          return 4;
        case 0x24:
          return 4;
        case 0x26:
          return 6;
        case 0x53:
          return 3;
        case 0x58:
          return 3;
        case 0x5b:
          return 3;
        case 0x76:
          return 4;
        case 0x82:
          return 4;
        case 0xc0:
          return 6;
        case 0xc4:
          return 3;
        case 0xc5:
          return 3;
        case 0xc6:
          return 3;
        case 0xe8:
          return 8;
        default:
          return 4;
      }
      break;
    default:
      return 4;
  }
}


// Handle commands recived from Serial.
// Supported following commands:
// 1. getinfo - prints some statistic from battery
// 2. ANY battery's command HEX string - prints
//    response from battery for the command.
//    Each HEX should be separated by space
//    For example: FF 03 4A 00 02
void handleCommandsFromSerial() {
  if (Serial.available()) {
    String cmd = Serial.readString();
    if (cmd.indexOf(F("getinfo")) != -1)
      printBatteryInfo();
    else if (cmd.indexOf(F("status")) != -1)
      WireBat.printStatus(Serial);
    else {
      Serial.print(F("Sending command to battery:"));
      byte command[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
      for (int i = 0; i < cmd.length(); i = i + 3) {
        String subCmd = cmd.substring(i, i + 2);
        byte b = strHexToByte(subCmd.c_str());
        command[i / 3] = b;
        Serial.print(F(" "));
        Serial.print(byteToStrHex(b));
      }
      Serial.println();
      readData(command);
    }
  }
}

// Prints some statistic from battery to Serial
void printBatteryInfo() {
  Serial.print(F("Device Name="));
  Serial.print(readString(Cmd_DeviceName));
  int batteryStatus = readParameter(Cmd_BatteryStatus);
  Serial.print(F(", Battery Status="));
  Serial.print(byteToStrHex(batteryStatus));
  if (batteryStatus == 0x84)
    Serial.print(F("(locked)"));
  else if (batteryStatus == 0x04)
    Serial.print(F("(good)"));
  Serial.print(F(", Undervoltage Count="));
  Serial.print(readSubParameter(Cmd_UndervoltageCount));
  Serial.print(F(", Temp="));
  Serial.print(getTemprature());
  Serial.print(F(", Cycles="));
  Serial.print(getCycleCount());
  Serial.println();

  Serial.print(F("Voltage="));
  Serial.print(getVoltage(Cmd_Voltage));
  Serial.print(F("v"));
  Serial.print(F(", V1="));
  Serial.print(getVoltage(Cmd_VoltageBank1));
  Serial.print(F("v"));
  Serial.print(F(", V2="));
  Serial.print(getVoltage(Cmd_VoltageBank2));
  Serial.print(F("v"));
  Serial.print(F(", V3="));
  Serial.print(getVoltage(Cmd_VoltageBank3));
  Serial.print(F("v"));
  Serial.print(F(", V4="));
  Serial.print(getVoltage(Cmd_VoltageBank4));
  Serial.print(F("v"));
  Serial.println();

  Serial.print(F("Charge Percent="));
  Serial.print(readParameter(Cmd_RelativeStateOfCharge));
  Serial.print(F("%"));
  Serial.print(F(", Capacity="));
  Serial.print(readParameter(Cmd_Capacity));
  Serial.print(F("mAH"));
  Serial.print(F(", Remaining Capacity="));
  Serial.print(readParameter(Cmd_RemainingCapacity));
  Serial.print(F("mAH"));
  Serial.print(F(", Current="));
  Serial.print(getCurrent(Cmd_Current));
  Serial.print(F("mA"));
  Serial.println();
}

// Calculates CRC for standart battery responses
// to commands like 0x07. It's allows to determine
// when main response bytes and don't need to collect
// other bytes that repeats CRC
byte calcCRC(byte data[]) {
  int sum = 0;
  for (int i = 0; i < crcBufLen; i++) {
    sum = sum + (int)data[i];
  }
  int crc = sum & 0xFF;

  return (byte)crc;
}

// Reads 2 byte value for standart one-byte command like 0x07
int readParameter(byte command) {
  beginTransmission(BATTERY_ADDRESS);
  writeByte(command);
  endTransmission(false);
  uint8_t vals[2] = {0, 0};
  uint8_t count = 0;
  int status = requestFrom(BATTERY_ADDRESS);
  if (status == 0) {
    vals[0] = readByte();
    vals[1] = readByte(true);
  }
  endTransmission(true);

  int val = (uint16_t)(vals[1]) << 8 | (uint16_t)(vals[0]);
  return val;
}

// Reads 2 bytes value for non-standart command like FF 03 4A 00 02
int readSubParameter(byte command) {
  beginTransmission(BATTERY_ADDRESS);
  writeByte(byte(0xFF));
  writeByte(byte(0x03));
  writeByte(command);
  writeByte(byte(0x00));
  writeByte(byte(2));
  endTransmission(false);
  uint8_t vals[2] = {0, 0};
  uint8_t count = 0;
  int status = requestFrom(BATTERY_ADDRESS);
  if (status == 0) {
    vals[0] = readByte();
    vals[1] = readByte(true);
  }
  endTransmission(true);
  int val = (uint16_t)(vals[0]) << 8 | (uint16_t)(vals[1]);
  return val;
}

// Reads multiple bytes value for any command
// and prints response to Serial
void readData(byte command[8]) {
  byte i2cBuffer[crcBufLen];
  for (int i = 0; i < crcBufLen; i++) i2cBuffer[i] = 0;
  uint8_t bufCount = 0;
  int status = beginTransmission(BATTERY_ADDRESS);
  Serial.print(F("status="));
  Serial.println(status);
  if (status == 0) {
    for (int i = 0; i < 8; i++)
      if (status == 0 && (command[i] > 0 || (i < 7 && command[i + 1] > 0))) {
        status = writeByte(command[i]);
        Serial.print(i);
        Serial.print(F(". write byte="));
        Serial.print(command[i]);
        Serial.print(F(", status="));
        Serial.println(status);
        i2cBuffer[bufCount++] = command[i];
      }
  }
  endTransmission(false);

  status = requestFrom(BATTERY_ADDRESS);
  if (status == 0) {
    uint8_t count = 0;
    while (count < 32) {
      byte b = readByte(count == 31);
      count++;
      if (calcCRC(i2cBuffer) == b) {
        readByte(true);
        Serial.print(F("CRC="));
        Serial.print(byteToStrHex(b));
        Serial.println();
        break;
      }
      else if (count > 2 && b == i2cBuffer[bufCount - 1]) {
        readByte(true);
        break;
      }
      else {
        Serial.print(F("count="));
        Serial.print(count);
        Serial.print(F("; value="));
        Serial.print(byteToStrHex(b));
        Serial.print(F(", "));
        Serial.print(b, DEC);
        Serial.print(F(" DEC, "));
        Serial.print(b, BIN);
        Serial.println(F(" BIN"));
        i2cBuffer[bufCount++] = b;
      }
    }
  }
  endTransmission(true);
}

String readString(byte command) {
  char i2cBuffer[responseBufLen];
  for (int i = 0; i < responseBufLen; i++) i2cBuffer[i] = 0;
  beginTransmission(BATTERY_ADDRESS);
  writeByte(command);
  endTransmission(false);
  int status = requestFrom(BATTERY_ADDRESS);
  if (status == 0) {
    int len = readByte();
    for (int i = 0; i < len; i++) {
      i2cBuffer[i] = readByte(i == len - 1);
    }
  }
  endTransmission(true);
  return String(i2cBuffer);
}

// Returns voltage in volts
float getVoltage(byte command) {
  int volts = readParameter(command);
  return (float)volts / 1000;
}

// Returns current in mA
int getCurrent(byte command) {
  int current = readParameter(command);
  return current;
}

// Returns temprature in Celsius
float getTemprature() {
  uint16_t temp = readParameter(Cmd_Temperature);

  temp = temp - 2730;   // Convert from Kelvin to Celcius

  float tempF = (float)temp / 10;

  return tempF;
}

// Returns discharge\charge cycles count
int getCycleCount() {
  return readParameter(Cmd_Cycles);
}

bool deviceAvailable(byte address) {
  int status = beginTransmission(address);
  endTransmission(true);
  return status == 0;
}

// Returns each address of devices connected to battery's I2C
void findBatteryDevices() {
  int nDevices = 0;
  for (int address = 1; address < 127; address++ )
  {
    // Use the return value of i2c.beginTransmission to see if
    // a device did acknowledge to the address.
    // Note that when using the standard Arduino Wire library you would check Wire.endTransmission.

    // If no error was reported, show the device we found!
    if (deviceAvailable(address))
    {
      Serial.print(F("I2C device found at address "));
      Serial.print(byteToStrHex(address));
      Serial.println(F(" !"));

      nDevices++;
    }
  }

  if (nDevices == 0)
  {
    Serial.println(F("No I2C devices found from battery side.\n"));
  }
  else
  {
    Serial.print(F("Found "));
    Serial.print(nDevices);
    Serial.println(F(" devices from battery side!\n"));
  }
}

// Converts HEX string to byte
byte strHexToByte(char *s)
{
  byte x = 0;
  for (;;) {
    char c = *s;
    if (c >= '0' && c <= '9') {
      x *= 16;
      x += c - '0';
    }
    else if (c >= 'A' && c <= 'F') {
      x *= 16;
      x += (c - 'A') + 10;
    }
    else break;
    s++;
  }
  return x;
}

// Converts byte to HEX sring
String byteToStrHex(byte b) {
  String strValue = "0x";
  if (b <= 0x0F)
    strValue = strValue + "0";
  strValue = strValue + String(b, HEX);
  return strValue;
}
