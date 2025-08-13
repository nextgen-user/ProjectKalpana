#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_WE.h>

#define MPU9250_ADDR 0x68

// BMP280 on Wire1 (SDA1/SCL1)
Adafruit_BMP280 bmp(&Wire1);
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// MPU9250 on Wire (default SDA/SCL)
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  Serial.println("Initializing sensors...");

  // Init both I2C buses
  Wire.begin();     // For MPU9250
  Wire1.begin();    // For BMP280

  // --- BMP280 Setup ---
  Serial.println("Initializing BMP280 on SDA1/SCL1...");
  if (!bmp.begin(0x76, BMP280_CHIPID)) { // try 0x77 if needed
    Serial.println(F("Could not find a valid BMP280 sensor on SDA1/SCL1"));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  bmp_temp->printSensorDetails();

  // --- MPU9250 Setup ---
  Serial.println("Initializing MPU9250 on SDA/SCL...");
  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  } else {
    Serial.println("MPU9250 is connected");
  }

  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  } else {
    Serial.println("Magnetometer is connected");
  }

  Serial.println("Position MPU9250 flat for calibration...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Calibration done!");

  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  Serial.println("Setup complete!");
}

void loop() {
  // --- BMP280 Readings ---
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  Serial.println("=== BMP280 (SDA1/SCL1) ===");
  Serial.print("Temperature: ");
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print("Pressure: ");
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  // --- MPU9250 Readings ---
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);

  Serial.println("=== MPU9250 (SDA/SCL) ===");
  Serial.println("Acceleration (g):");
  Serial.print(gValue.x); Serial.print("  ");
  Serial.print(gValue.y); Serial.print("  ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: "); Serial.println(resultantG);

  Serial.println("Gyroscope (°/s):");
  Serial.print(gyr.x); Serial.print("  ");
  Serial.print(gyr.y); Serial.print("  ");
  Serial.println(gyr.z);

  // Serial.println("Magnetometer (µT):");
  // Serial.print(magValue.x); Serial.print("  ");
  // Serial.print(magValue.y); Serial.print("  ");
  // Serial.println(magValue.z);

  // Serial.print("Temperature: "); Serial.println(temp);

  Serial.println("****TEAM_KALPANA_SENSOR_TESTING_TEENSY4.1****");
  delay(1000);
}
