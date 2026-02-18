#include <ModbusMaster.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>

// --- การตั้งค่าสถานะ (Custom Defines) ---
#define ON  0
#define OFF 1

#define SLAVE_ID 1
#define TX_PIN 21
#define RX_PIN 20 

#define PIXEL_PIN 7
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

ModbusMaster node;
Adafruit_AHTX0 aht;
Adafruit_INA219 ina219;

// ฟังก์ชันส่งค่า Float 32-bit ไปยัง Modbus
void writeFloatToLW(uint16_t start_reg, float value) {
  uint32_t temp;
  memcpy(&temp, &value, sizeof(float));
  
  uint16_t lowWord = (uint16_t)(temp & 0xFFFF);
  uint16_t highWord = (uint16_t)(temp >> 16);

  node.clearTransmitBuffer();
  node.setTransmitBuffer(0, lowWord);
  node.setTransmitBuffer(1, highWord);
  
  node.writeMultipleRegisters(start_reg, 2); 
}

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(50);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // เริ่มต้นสีน้ำเงิน
  pixels.show();
  
  Wire.begin(); 
  if (!aht.begin()) Serial.println("AHT10/20 Error!");
  if (!ina219.begin()) Serial.println("INA219 Error!");

  // ตั้งค่า Relay เป็น Output
  pinMode(2, OUTPUT); 
  pinMode(3, OUTPUT);
  // เริ่มต้นปิด Relay (สมมติ Relay Active Low ให้ใส่ !ON หรือเขียน HIGH ตามโมดูล)
  digitalWrite(2, OFF); 
  digitalWrite(3, OFF);

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

    // 1. ส่งค่า Sensor (Float) ไปหน้าจอ
    writeFloatToLW(100, t.temperature);
    writeFloatToLW(102, h.relative_humidity);
    writeFloatToLW(104, busVoltage);
    writeFloatToLW(106, current_mA);

    // 2. ส่งสถานะปุ่มกด (Digital Input) ไปที่หน้าจอ (LB 0, 1)
    // ใช้ logic: ถ้ากดปุ่ม (0) ให้ส่งสถานะ ON (1)
    node.writeSingleCoil(0, digitalRead(0) == LOW ? ON : OFF); 
    node.writeSingleCoil(1, digitalRead(1) == LOW ? ON : OFF); 

    // 3. อ่านค่าจากหน้าจอ (LB 2, 3) มาคุม Relay
    uint8_t result = node.readCoils(2, 2); 
    
    if (result == node.ku8MBSuccess) {
      // แยกบิตข้อมูลจาก Buffer
      uint8_t relay1_state = (node.getResponseBuffer(0) & 0x01);
      uint8_t relay2_state = (node.getResponseBuffer(0) >> 1) & 0x01;

      // สั่งงานขา Output ตามสถานะจากหน้าจอ
      digitalWrite(2, relay1_state == ON ? HIGH : LOW); 
      digitalWrite(3, relay2_state == ON ? HIGH : LOW); 
      
      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // สื่อสารสำเร็จสีเขียว
      pixels.show();
      ledActiveTime = millis();
    } else {
      // หากสื่อสารล้มเหลว (Error) แสดงสีแดง
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); 
      pixels.show();
    }

    lastUpdate = millis();
  }

  // เอฟเฟกต์ไฟหายใจ (Breathe Effect) เมื่อไม่มีการรับส่งข้อมูล
  if (millis() - ledActiveTime > 300) {
    float breath = (sin(millis() * 0.004) * 127) + 128;
    pixels.setPixelColor(0, pixels.Color(0, 0, (int)breath / 2)); 
    pixels.show();
  }
}
