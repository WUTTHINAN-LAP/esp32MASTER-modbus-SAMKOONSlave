#include <ModbusMaster.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>

#define SLAVE_ID 1
#define TX_PIN 21
#define RX_PIN 20 

#define PIXEL_PIN 7
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

ModbusMaster node;
Adafruit_AHTX0 aht;
Adafruit_INA219 ina219;

// ฟังก์ชันส่งค่า Float 32-bit สำหรับ ModbusMaster Library
void writeFloatToLW(uint16_t start_reg, float value) {
  uint32_t temp;
  memcpy(&temp, &value, sizeof(float));
  
  uint16_t lowWord = (uint16_t)(temp & 0xFFFF);
  uint16_t highWord = (uint16_t)(temp >> 16);

  node.clearTransmitBuffer();              // ล้าง Buffer ก่อนส่ง
  node.setTransmitBuffer(0, lowWord);      // ใส่ค่าที่ 1 (LW100)
  node.setTransmitBuffer(1, highWord);     // ใส่ค่าที่ 2 (LW101)
  
  // สั่งเขียนไปยัง Slave: เริ่มที่ start_reg, จำนวน 2 registers
  node.writeMultipleRegisters(start_reg, 2); 
}

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(50);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
  pixels.show();
  
  Wire.begin(); 
  if (!aht.begin()) Serial.println("AHT10/20 Error!");
  if (!ina219.begin()) Serial.println("INA219 Error!");

  pinMode(2, OUTPUT); 
  pinMode(3, OUTPUT);
  pinMode(0, INPUT_PULLUP); 
  pinMode(1, INPUT_PULLUP);

  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  node.begin(SLAVE_ID, Serial1);
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long ledActiveTime = 0;

  if (millis() - lastUpdate > 200) {
    sensors_event_t h, t;
    aht.getEvent(&h, &t); 
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();

    // ส่งค่าไปที่ LW 100, 102, 104, 106
    writeFloatToLW(100, t.temperature);
    writeFloatToLW(102, h.relative_humidity);
    writeFloatToLW(104, busVoltage);
    writeFloatToLW(106, current_mA);

    // ส่งสถานะปุ่มไป LB 0, 1
    node.writeSingleCoil(0, !digitalRead(0)); 
    node.writeSingleCoil(1, !digitalRead(1)); 

    // อ่านค่าจากหน้าจอ LB 2, 3 มาคุม Relay
    uint8_t result = node.readCoils(2, 2); 
    
    if (result == node.ku8MBSuccess) {
      digitalWrite(2, node.getResponseBuffer(0) & 0x01); 
      digitalWrite(3, (node.getResponseBuffer(0) >> 1) & 0x01); 
      
      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
      pixels.show();
      ledActiveTime = millis();
    } else {
      // หากสื่อสารล้มเหลว (เช่น สายหลุด) แสดงสีแดง
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); 
      pixels.show();
    }

    lastUpdate = millis();
  }

  // เอฟเฟกต์ไฟหายใจ
  if (millis() - ledActiveTime > 300) {
    float breath = (sin(millis() * 0.004) * 127) + 128;
    pixels.setPixelColor(0, pixels.Color(0, 0, (int)breath / 2)); 
    pixels.show();
  }
}