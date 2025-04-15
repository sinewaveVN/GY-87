#include <GY87.h>

// Khởi tạo GY87 với SDA = GPIO4, SCL = GPIO3
GY87 sensor(4, 3);

#define LED_PIN 2
bool blinkState = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);

  // Khởi động cảm biến
  if (sensor.begin()) {
    Serial.println("GY-87 khoi tao thanh cong!");
  } else {
    Serial.println("Loi khoi tao GY-87!");
    while (1);
  }

  // Nhãn cho Serial Plotter
  Serial.println("ax\tay\taz\tgx\tgy\tgz\ttemp\tpress\theading");
}

void loop() {
  // Đọc dữ liệu
  if (sensor.read()) {
    // In dữ liệu cho Serial Plotter
    Serial.print(sensor.ax); Serial.print("\t");
    Serial.print(sensor.ay); Serial.print("\t");
    Serial.print(sensor.az); Serial.print("\t");
    Serial.print(sensor.gx); Serial.print("\t");
    Serial.print(sensor.gy); Serial.print("\t");
    Serial.print(sensor.gz); Serial.print("\t");
    Serial.print(sensor.temperature); Serial.print("\t");
    Serial.print(sensor.pressure); Serial.print("\t");
    Serial.print(sensor.heading); Serial.println();
  } else {
    Serial.println("Loi doc du lieu!");
  }

  // Nháy LED
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  delay(100); // Cập nhật 10 lần/giây
}