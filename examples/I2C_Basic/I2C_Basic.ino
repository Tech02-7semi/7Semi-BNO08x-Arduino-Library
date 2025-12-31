/*  I2C_Basic.ino  —  7Semi BNO08x (SHTP) example over I²C
 *
 *  What this does
 *  - Creates an I²C transport (default 0x4B, 400 kHz)
 *  - Starts the BNO08x
 *  - Enables several sensor reports at ~100 Hz
 *  - In loop(), calls processData() to read+decode packets (if any)
 *  - Prints the most recent values every 250 ms
 *
 *  Wiring (typical 3V3 MCU)
 *    VDD -> 3V3
 *    GND -> GND
 *    SDA -> MCU SDA
 *    SCL -> MCU SCL
 *    PS0 -> LOW
 *    PS1 -> LOW
 *
 *  Notes
 *  - If your board is at 0x4A instead of 0x4B, change the constructor below.
 */
#define BNO_USE_I2C
#include <7Semi_BNO08x.h>  // main driver (BNO08x_7Semi + SH2_* IDs)


// ---------- Transport & device ----------
// Many BNO08x boards use 0x4B (sometimes 0x4A). Start at 400 kHz.
//For Arduino
int8_t BNO_SDA = -1; 
int8_t BNO_SCL = -1;
uint32_t BNO_clock = 400000;
// int8_t BNO_SDA = 21; 
// int8_t BNO_SCL = 22; 
static BnoI2CBus bus(Wire, BNO_SDA, BNO_SCL, 0x4B, BNO_clock);
static BNO08x_7Semi bno(bus);

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

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB */
  }
  Serial.println(F("\n[7Semi BNO08x — I2C_Basic]"));

  if (!bno.begin()) {
    Serial.println(" BNO08x initialization failed");
    while (1) delay(1000);
  }

  if (!bno.enableReport(ACCELEROMETER, 20))
    Serial.println(" Accelerometer failed");

  if (!bno.enableReport(GYROSCOPE_CALIBRATED, 20))
    Serial.println(" Gyroscope failed");

  if (!bno.enableReport(MAGNETIC_FIELD_CALIBRATED, 50))
    Serial.println(" Magnetometer failed");

  if (!bno.enableReport(ROTATION_VECTOR, 20))
    Serial.println(" Rotation Vector failed");

  if (!bno.enableReport(LINEAR_ACCELERATION, 20))
    Serial.println(" Linear Acceleration failed");

  if (!bno.enableReport(GRAVITY, 20))
    Serial.println(" Gravity failed");
}

/* ---------------- Loop ---------------- */
void loop() {
  bno.processData();  // Always run as fast as possible
  d.valid = 0;
  d.valid |= bno.getAccelerometer(d.ax, d.ay, d.az);
  d.valid |= bno.getGyroscope(d.gx, d.gy, d.gz);
  d.valid |= bno.getMagnetometer(d.mx, d.my, d.mz);
  d.valid |= bno.getQuaternion(d.qi, d.qj, d.qk, d.qr);
  d.valid |= bno.getLinearAccel(d.lax, d.lay, d.laz);
  d.valid |= bno.getGravity(d.grx, d.gry, d.grz);

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
