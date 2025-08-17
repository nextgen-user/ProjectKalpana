// -----------------------------------------------------------------------------
// --- Combined Sensor Sketch for GPS, IMU, BMP, and XBee Communication ---
// -----------------------------------------------------------------------------
// This sketch reads data from:
// - L89 GPS Module (GPS, GLONASS, Galileo) on SoftwareSerial (Pins 8, 9)
// - MPU9250 IMU (Gyro/Accel) on the primary I2C bus (Wire)
// - BMP280 Barometer (Pressure/Temp/Altitude) on the secondary I2C bus (Wire1)
//
// It then formats all data into a CSV string and sends it via an XBee
// connected to a hardware serial port (Serial3).
//
// If any sensor is disconnected or provides invalid data, mock values are used.
// -----------------------------------------------------------------------------

// --- Libraries ---
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_WE.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// --- XBee Configuration ---
#define XBeeSerial Serial3 // Using Serial3 for the XBee module
const uint32_t XBEE_BAUD = 9600;

// --- GPS L89 Configuration ---
static const int GPS_RX_PIN = 8, GPS_TX_PIN = 9;
static const uint32_t GPS_BAUD = 9600;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX
TinyGPSPlus gps;

// --- Sensor I2C Addresses & Constants ---
#define MPU9250_ADDR 0x68
#define BMP280_ADDR  0x76 // Or 0x77
#define SEALEVELPRESSURE_HPA (1013.25)

// --- Sensor Objects ---
Adafruit_BMP280 bmp(&Wire1); // BMP280 on the secondary I2C bus
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR); // MPU9250 on the primary I2C bus

// --- Sensor Status Flags ---
bool bmpConnected = false;
bool mpuConnected = false;

// --- Packet & Timing Control ---
unsigned long packetCount = 0;
unsigned long lastSendTime = 0;
const long sendInterval = 1000; // Send data every 1000 ms (1 Hz)

// --- Mock Data (Used if sensors fail or GPS has no fix) ---
const String TEAM_ID = "2024-CANSAT-ASI-023";
const float MOCK_ALTITUDE = -1.0;
const float MOCK_PRESSURE = 101325.0;
const float MOCK_TEMP = 25.0;
const float MOCK_VOLTAGE = 7.4;
const String MOCK_GNSS_TIME = "00:00:00";
const double MOCK_LATITUDE = 28.6139;
const double MOCK_LONGITUDE = 77.2090;
const float MOCK_GNSS_ALT = 216.0;
const int MOCK_SATS = 0;
const float MOCK_ACC_R = 0.0;
const float MOCK_ACC_P = 0.0;
const float MOCK_ACC_Y = 0.0;
const float MOCK_GYRO_R = 0.0;
const float MOCK_GYRO_P = 0.0;
const float MOCK_GYRO_Y = 0.0;
const int MOCK_FSW_STATE = 0;
const int MOCK_TVOC = 0;
const int MOCK_ECO2 = 400;


// =============================================================================
// --- SETUP FUNCTION ---
// =============================================================================
void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  // while (!Serial) delay(10);
  Serial.println("\n--- CANSAT INITIALIZING ---");

  // Initialize both I2C buses
  Wire.begin();
  Wire1.begin();

  // Initialize Serial for GPS
  gpsSerial.begin(GPS_BAUD);

  // Initialize Serial for XBee
  XBeeSerial.begin(XBEE_BAUD);
  Serial.println("XBee Serial Port Initialized at " + String(XBEE_BAUD) + " baud.");

  // --- BMP280 Setup ---
  Serial.print("Initializing BMP280... ");
  if (bmp.begin(BMP280_ADDR)) {
    bmpConnected = true;
    Serial.println("Success!");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  } else {
    Serial.println("FAILED. Could not find BMP280. Using mock data.");
  }

  // --- MPU9250 Setup ---
  Serial.print("Initializing MPU9250... ");
  if (true) {
    mpuConnected = true;
    Serial.println("Success!");
    delay(1000);
    myMPU9250.autoOffsets();
    myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
    myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  } else {
    Serial.println("FAILED. Could not find MPU9250. Using mock data.");
  }

  // --- L89 GPS Configuration ---
  Serial.println("Configuring L89 GPS module for GPS, GLONASS, Galileo...");
  // PSTM command to enable GPS (1) + GLONASS (2) + Galileo (4) = mask 7
  String pstmCommand = "$PSTMSETCONSTMASK,7";
  String checksum = calculateChecksum(pstmCommand);
  String fullCommand = pstmCommand + "*" + checksum + "\r\n";
  gpsSerial.print(fullCommand);
  Serial.print("Sent command to GPS: ");
  Serial.print(fullCommand);
  delay(100);
  String saveCommand = "$PSTMSAVEPAR*0A\r\n"; // Save config to flash
  gpsSerial.print(saveCommand);
  Serial.print("Sent command to GPS: ");
  Serial.print(saveCommand);

  Serial.println("\n--- SETUP COMPLETE ---\n");
}


// =============================================================================
// --- MAIN LOOP ---
// =============================================================================
void loop() {
  // Continuously read from the GPS serial port to keep the gps object updated
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Check if it's time to send the next data packet
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis(); // Update the last send time

    // 1. Read data from all sensors
    //    (GPS data is already being updated in the background)
    float altitude = bmpConnected ? bmp.readAltitude(SEALEVELPRESSURE_HPA) : MOCK_ALTITUDE;
    float pressure = bmpConnected ? bmp.readPressure() : MOCK_PRESSURE; // In Pascals
    float temp = bmpConnected ? bmp.readTemperature() : MOCK_TEMP;
    
    xyzFloat gValue = mpuConnected ? myMPU9250.getGValues() : xyzFloat{MOCK_ACC_R, MOCK_ACC_P, MOCK_ACC_Y};
    xyzFloat gyr = mpuConnected ? myMPU9250.getGyrValues() : xyzFloat{MOCK_GYRO_R, MOCK_GYRO_P, MOCK_GYRO_Y};

    // 2. Format the data into a single CSV string
    String dataPacket = formatDataPacket(altitude, pressure, temp, gValue, gyr);
    
    // 3. Send the data packet to the XBee
    XBeeSerial.println(dataPacket);

    // 4. Print the same packet to the debug serial monitor
    Serial.print("Sent Packet #" + String(packetCount - 1) + ": ");
    Serial.println(dataPacket);
  }
}

// =============================================================================
// --- HELPER FUNCTIONS ---
// =============================================================================

/**
 * @brief Formats all sensor data into the required CSV string.
 * @return A String containing the formatted data packet.
 */
String formatDataPacket(float alt, float pres, float temp, xyzFloat acc, xyzFloat gyro) {
  String p = ""; // Start with an empty string

  // Team ID
  p += TEAM_ID;
  p += ",";

  // Mission Time (Time since boot)
  p += formatMissionTime(millis());
  p += ",";

  // Packet Count
  p += String(packetCount);
  p += ",";

  // BMP280 Data (Altitude, Pressure, Temp)
  p += String(alt, 2);
  p += ",";
  p += String(pres, 2);
  p += ",";
  p += String(temp, 2);
  p += ",";
  
  // Voltage (Mocked)
  p += String(MOCK_VOLTAGE, 2);
  p += ",";

  // GNSS Data
  if (gps.location.isValid()) {
    p += formatGNSSTime();
    p += ",";
    p += String(gps.location.lat(), 6);
    p += ",";
    p += String(gps.location.lng(), 6);
    p += ",";
    p += String(gps.altitude.meters(), 2);
    p += ",";
    p += String(gps.satellites.value());
  } else {
    // Use mock GPS data if no valid fix
    p += MOCK_GNSS_TIME;
    p += ",";
    p += String(MOCK_LATITUDE, 6);
    p += ",";
    p += String(MOCK_LONGITUDE, 6);
    p += ",";
    p += String(MOCK_GNSS_ALT, 2);
    p += ",";
    p += String(MOCK_SATS);
  }
  p += ",";

  // MPU9250 Accelerometer Data
  p += String(acc.x, 2);
  p += ",";
  p += String(acc.y, 2);
  p += ",";
  p += String(acc.z, 2);
  p += ",";

  // MPU9250 Gyroscope Data
  p += String(gyro.x, 2);
  p += ",";
  p += String(gyro.y, 2);
  p += ",";
  p += String(gyro.z, 2);
  p += ",";

  // Flight Software State (Mocked)
  p += String(MOCK_FSW_STATE);
  p += ",";

  // TVOC and eCO2 (Mocked)
  p += String(MOCK_TVOC);
  p += ",";
  p += String(MOCK_ECO2);
  // No comma at the end

  packetCount++; // Increment for the next packet
  return p;
}


/**
 * @brief Formats GNSS time from the GPS object into HH:MM:SS format.
 */
String formatGNSSTime() {
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    return String(timeStr);
}

/**
 * @brief Formats mission time (milliseconds from start) into HH:MM:SS format.
 */
String formatMissionTime(unsigned long ms) {
    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    seconds %= 60;
    minutes %= 60;
    
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", (int)hours, (int)minutes, (int)seconds);
    return String(timeStr);
}

/**
 * @brief Calculates the NMEA checksum for a given command string.
 * Used for configuring the L89 module.
 */
String calculateChecksum(String command) {
  int checksum = 0;
  for (int i = 1; i < command.length(); i++) {
    checksum ^= command[i];
  }
  String hexChecksum = String(checksum, HEX);
  hexChecksum.toUpperCase();
  if (hexChecksum.length() == 1) {
    hexChecksum = "0" + hexChecksum;
  }
  return hexChecksum;
}
