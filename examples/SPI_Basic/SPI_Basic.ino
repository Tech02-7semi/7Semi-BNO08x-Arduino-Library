/******************************************************************************
 * File    : SPI_Basic.ino
 * Module  : 7Semi BNO085 — SPI Basic Example (ESP32)
 * Version : 0.1.0
 * License : MIT
 *
 * Summary
 * -------
 * Minimal SPI example for the BNO085 (SHTP protocol) on ESP32 using the
 * 7Semi_BNO08x driver and the BnoSPIBus transport adapter.
 *
 * Transport Notes
 * ---------------
 * - SPI Mode    : MODE 3
 * - Bit order   : MSB first
 * - Clock       : ≤ 3 MHz (BNO085 spec)
 * - INTN pin    : Optional for SPI transport
 * - RST pin     : Optional for SPI transport
 *
 * Output
 * ------
 * - Accelerometer (m/s²)
 * - Gyroscope (rad/s)
 * - Magnetometer (µT)
 * - Quaternion (i j k r)
 * - Linear acceleration (m/s²)
 * - Gravity (m/s²)
 ******************************************************************************/
#include <7Semi_BNO08x.h>
#define BNO_USE_SPI

/* ---------------- Pins ---------------- */
constexpr int PIN_SCK = -1;
constexpr int PIN_MISO = -1;
constexpr int PIN_MOSI = -1;
constexpr int PIN_CS = 10;
constexpr int PIN_INTN = 9;  // Optional for SPI
constexpr int PIN_RST = 8;   // Optional for SPI

// constexpr int PIN_SCK = 18;
// constexpr int PIN_MISO = 19;
// constexpr int PIN_MOSI = 23;
// constexpr int PIN_CS = 5;
// constexpr int PIN_INTN = 4;  // Optional for SPI
// constexpr int PIN_RST = 17;  // Optional for SPI

/* ---------------- SPI Bus ---------------- */
BnoSPIBus bus(
  SPI,
  PIN_CS,
  PIN_INTN,
  PIN_RST,
  3000000UL,
  SPI_MODE3,
  PIN_SCK,
  PIN_MISO,
  PIN_MOSI);

/* ---------------- IMU ---------------- */
BNO08x_7Semi imu(bus);

/* ---------------- Cached IMU Data ---------------- */
struct ImuData {
  float ax, ay, az;      // Accelerometer
  float gx, gy, gz;      // Gyroscope
  float mx, my, mz;      // Magnetometer
  float qi, qj, qk, qr;  // Rotation Vector
  float lax, lay, laz;   // Linear acceleration
  float grx, gry, grz;   // Gravity
  bool valid;
};

ImuData d;

/* ---------------- Setup ---------------- */
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  Serial.println("\n[BNO08x SPI ESP32]");

  if (!imu.begin()) {
    Serial.println(" BNO08x initialization failed");
    while (1) delay(1000);
  }

  if (!imu.enableReport(ACCELEROMETER, 20))
    Serial.println(" Accelerometer failed");

  if (!imu.enableReport(GYROSCOPE_CALIBRATED, 20))
    Serial.println(" Gyroscope failed");

  if (!imu.enableReport(MAGNETIC_FIELD_CALIBRATED, 50))
    Serial.println(" Magnetometer failed");

  if (!imu.enableReport(ROTATION_VECTOR, 20))
    Serial.println(" Rotation Vector failed");

  if (!imu.enableReport(LINEAR_ACCELERATION, 20))
    Serial.println(" Linear Acceleration failed");

  if (!imu.enableReport(GRAVITY, 20))
    Serial.println(" Gravity failed");
}

/* ---------------- Loop ---------------- */
void loop() {
  imu.processData();  // Always run as fast as possible
  d.valid = 0;
  d.valid |= imu.getAccelerometer(d.ax, d.ay, d.az);
  d.valid |= imu.getGyroscope(d.gx, d.gy, d.gz);
  d.valid |= imu.getMagnetometer(d.mx, d.my, d.mz);
  d.valid |= imu.getQuaternion(d.qi, d.qj, d.qk, d.qr);
  d.valid |= imu.getLinearAccel(d.lax, d.lay, d.laz);
  d.valid |= imu.getGravity(d.grx, d.gry, d.grz);

  static uint32_t t = 0;
  if (!d.valid || millis() - t < 250) return;
  t = millis();
  
  Serial.print("ACC : ");
  Serial.print(d.ax, 2);
  Serial.print(' ');
  Serial.print(d.ay, 2);
  Serial.print(' ');
  Serial.print(d.az, 2);
  Serial.println(" m/s²");

  Serial.print("GYRO: ");
  Serial.print(d.gx, 2);
  Serial.print(' ');
  Serial.print(d.gy, 2);
  Serial.print(' ');
  Serial.print(d.gz, 2);
  Serial.println(" rad/s");

  Serial.print("MAG : ");
  Serial.print(d.mx, 1);
  Serial.print(' ');
  Serial.print(d.my, 1);
  Serial.print(' ');
  Serial.print(d.mz, 1);
  Serial.println(" uT");

  Serial.print("RV  : ");
  Serial.print(d.qi, 3);
  Serial.print(' ');
  Serial.print(d.qj, 3);
  Serial.print(' ');
  Serial.print(d.qk, 3);
  Serial.print(' ');
  Serial.println(d.qr, 3);

  Serial.print("LIN : ");
  Serial.print(d.lax, 2);
  Serial.print(' ');
  Serial.print(d.lay, 2);
  Serial.print(' ');
  Serial.print(d.laz, 2);
  Serial.println(" m/s²");

  Serial.print("GRAV: ");
  Serial.print(d.grx, 2);
  Serial.print(' ');
  Serial.print(d.gry, 2);
  Serial.print(' ');
  Serial.println(d.grz, 2);

  Serial.println();
}
