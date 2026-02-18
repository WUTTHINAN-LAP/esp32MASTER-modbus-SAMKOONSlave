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
  while (!Serial) delay(10); // รอจนกว่า Serial Monitor จะเปิด

  Serial.println("\n--- System Starting ---");

  pixels.begin();
  pixels.setBrightness(50);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
  pixels.show();
  
  Wire.begin(); 
  if (!aht.begin()) {
    Serial.println("[-] AHT10/20 Error!");
  } else {
    Serial.println("[+] AHT10/20 Connected.");
  }

  if (!ina219.begin()) {
    Serial.println("[-] INA219 Error!");
  } else {
    Serial.println("[+] INA219 Connected.");
  }

  pinMode(2, OUTPUT); 
  pinMode(3, OUTPUT);
  digitalWrite(2, OFF); 
  digitalWrite(3, OFF);

  pinMode(0, INPUT_PULLUP); 
  pinMode(1, INPUT_PULLUP);

  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  node.begin(SLAVE_ID, Serial1);
  
  Serial.println("[+] Modbus Master Initialized on Serial1.");
  Serial.println("---------------------------------------\n");
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long ledActiveTime = 0;

  if (millis() - lastUpdate > 1000) { // ปรับเป็น 1 วินาทีเพื่อให้ Monitor อ่านทัน
    sensors_event_t h, t;
    aht.getEvent(&h, &t); 
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();

    // แสดงค่าออกทาง Serial Monitor
    Serial.println(">>>> SENSOR DATA <<<<");
    Serial.printf("Temp: %.2f C | Humid: %.2f %%\n", t.temperature, h.relative_humidity);
    Serial.printf("Voltage: %.2f V | Current: %.2f mA\n", busVoltage, current_mA);

    // 1. ส่งค่า Sensor ไปที่ HMI
    writeFloatToLW(100, t.temperature);
    writeFloatToLW(102, h.relative_humidity);
    writeFloatToLW(104, busVoltage);
    writeFloatToLW(106, current_mA);

    // 2. ส่งสถานะปุ่มกด (Invert ค่าตาม Pull-up)
    uint8_t btn0 = (digitalRead(0) == LOW) ? 1 : 0;
    uint8_t btn1 = (digitalRead(1) == LOW) ? 1 : 0;
    node.writeSingleCoil(0, btn0); 
    node.writeSingleCoil(1, btn1); 
    Serial.printf("Buttons -> BTN0: %s | BTN1: %s\n", btn0 ? "PRESSED" : "IDLE", btn1 ? "PRESSED" : "IDLE");

    // 3. อ่านค่าจากหน้าจอมาคุม Relay
    uint8_t result = node.readCoils(2, 2); 
    
    if (result == node.ku8MBSuccess) {
      uint8_t relay1_state = (node.getResponseBuffer(0) & 0x01);
      uint8_t relay2_state = (node.getResponseBuffer(0) >> 1) & 0x01;

      digitalWrite(2, relay1_state == 1 ? ON : OFF); 
      digitalWrite(3, relay2_state == 1 ? ON : OFF); 
      
      Serial.printf("HMI Control -> Relay1: %s | Relay2: %s [OK]\n", 
                    relay1_state ? "ON" : "OFF", relay2_state ? "ON" : "OFF");

      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
      pixels.show();
      ledActiveTime = millis();
    } else {
      Serial.printf("Modbus Error: 0x%02X\n", result);
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); 
      pixels.show();
    }
    
    Serial.println("---------------------------------------");
    lastUpdate = millis();
  }

  // Breathe Effect
  if (millis() - ledActiveTime > 300) {
    float breath = (sin(millis() * 0.004) * 127) + 128;
    pixels.setPixelColor(0, pixels.Color(0, 0, (int)breath / 2)); 
    pixels.show();
  }
}
