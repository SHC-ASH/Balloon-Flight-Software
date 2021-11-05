#include <Adafruit_BME280.h>
#include <Arduino_LSM6DS3.h>
#include <TinyGPS++.h>
#include <Wire.h>

// Constants
#define GPS_I2C_ADDR 0x42
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define VELOCITY_BUFFER_SIZE 5

#define LED_PIN 13
#define CAM_PIN 11
#define VENT_MOTOR 9
#define DROP_MOTOR 10

// Navigation Variables
float PRESSURE_MARGIN = 10.0f;
float MAX_SPEED = 1.0f;
float SPEED_MARGIN = 0.3f;


// GPS Variables
TinyGPSPlus gps;
bool haveNewGPSUpdate;
double gpsAltitude;
bool hasGpsLock;

// BME Variables
Adafruit_BME280 bme;
float pressureGround = 1013.25;
float bmeAltitude = 0;
float bmePressure = 0;
float bmeHumidity = 0;
float bmeTemperature = 0;

// IMU Variables
float gyro [3];
float acc [3];

// Timing Variables
int LOG_WAIT_TIME = 100;
int BME_WAIT_TIME = 100;
int IMU_WAIT_TIME = 100;
int VELOCITY_WAIT_TIME = 1000;
int LED_ON_TIME = 100;
int LED_OFF_TIME = 10000;
int CAM_TRIGGER_TIME = 100;
int CAM_WAIT_TIME = 60*1000;
int MARGIN_STABILITY_TIME = 60*1000;
int timeLastLog = 0;
int timeLastBmeRead = 0;
int timeLastImuRead = 0;
int timeLastGpsRead = 0;
int timeLastVelocityCalc = 0;
int timeLastLedUpdate = 0;
int timeLastCamUpdate = 0;
int timeEnteringPressureTarget = 0;

// Velocity Variables
float gpsVelocity = 0;
float pressureVelocity = 0;
float gpsVelBuffer [VELOCITY_BUFFER_SIZE];
float bmeVelBuffer [VELOCITY_BUFFER_SIZE];
float lastGpsAltitude = 0;
float lastBmeAltitude = 0;
int bufferCounter = 0;

// Status Variables
bool ventState;
bool dropState;
bool camState;
bool ledState;

// Flight Plan Definition
struct FlightPhase {
    int day;
    int hour;
    int minute;
    float target;
    bool idle;
};

FlightPhase activePhase;
int flightPhase;
int NUM_FLIGHT_PHASES = 5;

FlightPhase flightPlanTest[] {
  {5, 12, 40, 0, true},
  {5, 12, 41, 700, false},
  {5, 12, 42, 998, false},
  {5, 12, 43, 1050, false},
  {5, 12, 44, 0, true}
};

FlightPhase flightPlan[] {
  {5, 13, 0, 0, true},
  {5, 13, 10, 624, false},
  {5, 20, 0, 700, false},
  {6, 1, 797, false},
  {6, 4, 1030, false}

};

// Function Definitions
void displayInfo();
void readIMU();
void readBME();
void printGpsDate();
void updateMotors();
void logSensorData();
float calculate_max_speed();


void setup() 
{
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  pinMode(CAM_PIN, OUTPUT);
  pinMode(VENT_MOTOR, OUTPUT);
  pinMode(DROP_MOTOR, OUTPUT);
  digitalWrite(CAM_PIN, HIGH);
  camState = 1;
  
  bme.begin();
  IMU.begin();
  Serial.println("Sensors initialized!");
  delay(1000);

  // Get BME Data at least once
  readBME();

  // Print CSV Header
  Serial1.println("hardware time, utc time, altitude [bme], altitude [gps], latitude, longitude, satellite locks, pressure (hPa), humidity, temperature, acceleration (g) x, y, z, gyroscope (deg/sec) x, y, z, gps velocity, pressure velocity, ventState, dropState, flightPhase");
}

void loop() 
{
  Wire.requestFrom(GPS_I2C_ADDR, 1);    // request data from GPS module
  if (Wire.available()) 
  {
    char c = Wire.read(); // receive a byte as character
    haveNewGPSUpdate = gps.encode(c);
  }

  if (haveNewGPSUpdate)
  {
    gpsAltitude = gps.altitude.meters();
    hasGpsLock = gps.location.isValid();

    logSensorData();
    displayInfo();
    Serial.print(hasGpsLock); Serial.print(" "); Serial.print("GPS: "); Serial.print(gpsVelocity); Serial.print("    BME: "); Serial.println(pressureVelocity);
    timeLastGpsRead = millis();
  }

  if (millis() - timeLastLog >= LOG_WAIT_TIME)
  {
    logSensorData();
  }

  if (millis() - timeLastBmeRead >= BME_WAIT_TIME)
  {
    readBME();
  }

  if (millis() - timeLastImuRead >= IMU_WAIT_TIME)
  {
    readIMU();
  }

  if (millis() - timeLastVelocityCalc >= VELOCITY_WAIT_TIME)
  {
    if (timeLastVelocityCalc == 0)
    {
      timeLastVelocityCalc = millis();
      return;
    }

    float dT = (millis() - timeLastVelocityCalc) / 1000.0f;

    if (lastGpsAltitude == 0)
      lastGpsAltitude = gpsAltitude;
    if (lastBmeAltitude == 0)
      lastBmeAltitude = bmeAltitude;

    gpsVelBuffer[bufferCounter] = (gpsAltitude - lastGpsAltitude) / dT;
    bmeVelBuffer[bufferCounter] = (bmeAltitude - lastBmeAltitude) / dT;

    lastGpsAltitude = gpsAltitude;
    lastBmeAltitude = bmeAltitude;

    float velSum = 0;
    for (int i = 0; i < VELOCITY_BUFFER_SIZE; i++)
      velSum += gpsVelBuffer[i];    
    gpsVelocity = velSum / (float)VELOCITY_BUFFER_SIZE;

    velSum = 0;
    for (int i = 0; i < VELOCITY_BUFFER_SIZE; i++)
      velSum += bmeVelBuffer[i];
    pressureVelocity = velSum / (float)VELOCITY_BUFFER_SIZE;

    bufferCounter = (bufferCounter + 1) % VELOCITY_BUFFER_SIZE;

    timeLastVelocityCalc = millis();
  }

  if (ledState == true)
  {
      if (millis() - timeLastLedUpdate >= LED_ON_TIME)
      {
        ledState = false;
        timeLastLedUpdate = millis();
      }
  } else {    
      if (millis() - timeLastLedUpdate >= LED_OFF_TIME)
      {
        ledState = true && hasGpsLock;
        timeLastLedUpdate = millis();
      }
  }

  if (camState == true)
  {
      if (millis() - timeLastCamUpdate >= CAM_WAIT_TIME)
      {
        camState = false;
        timeLastCamUpdate = millis();
      }
  } else {    
      if (millis() - timeLastCamUpdate >= CAM_TRIGGER_TIME)
      {
        camState = true;
        timeLastCamUpdate = millis();
      }
  }

  ventState = false;
  dropState = false;

  activePhase = flightPlan[flightPhase];

  /*
  if (bmePressure > activePhase.target + PRESSURE_MARGIN && bmePressure < activePhase.target - PRESSURE_MARGIN)
  {
    timeEnteringPressureTarget = millis();
  }*/

  if (!activePhase.idle)// && (millis() - timeEnteringPressureTarget < MARGIN_STABILITY_TIME))
  {
    if (bmePressure < activePhase.target)
    {
      // We need to go down
      // If our velocity is greater than -3.0 m/s + margin, vent
      // Else if velocity is less than -3.0 - margin  drop ballast
      if (pressureVelocity > -calculate_max_speed() + SPEED_MARGIN)
      {
        // vent
        ventState = true;
        dropState = false;
      } else if (pressureVelocity < -calculate_max_speed() - SPEED_MARGIN) {
        // ballast
        ventState = false;
        dropState = true;
      }
    }
    if (bmePressure > activePhase.target)
    {
      // We need to go up
      // If our velocity is less than 3.0 m/s - margin, we need to drop ballast
      // If our velocity is greater than 3.0 m/s + margin, we need to vent
      if (pressureVelocity < calculate_max_speed() - SPEED_MARGIN)
      {
        // ballast
        ventState = false;
        dropState = true;
      } else if (pressureVelocity > calculate_max_speed() + SPEED_MARGIN) {
        // vent
        ventState = true;
        dropState = false;
      }
    }
  }

  updateMotors();

  if (flightPhase == NUM_FLIGHT_PHASES - 1)
  {
    MAX_SPEED = 5.0;
    return;
  }

  FlightPhase nextPhase = flightPlan[flightPhase+1];
  if (nextPhase.day == gps.date.day()
    && nextPhase.hour == gps.time.hour()
    && nextPhase.minute <= gps.time.minute())
    {
      flightPhase += 1;
    }

}

float calculate_max_speed()
{
  if (abs(bmePressure - activePhase.target) >= 30.0f)
  {
    return MAX_SPEED;
  } else {
    return (abs(bmePressure - activePhase.target) / 30.0f) * MAX_SPEED;
  }
}

void s() // Utility function that prints commas between the data
{
  Serial1.print(",");
}

void readIMU()
{
  //IMU - gyro and accelerometer readings
  if(IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);  //degrees per second
  } else {
    gyro[0] = 0.0;
    gyro[1] = 0.0;
    gyro[2] = 0.0;
  }

  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(acc[0], acc[1], acc[2]); //in g
  } else {
    acc[0] = 0.0;
    acc[1] = 0.0;
    acc[2] = 0.0;
  }

  timeLastImuRead = millis();
}

void readBME()
{
  bmeTemperature = bme.readTemperature();
  bmePressure = bme.readPressure() / 100.0f;
  bmeHumidity = bme.readHumidity();
  bmeAltitude = bme.readAltitude(pressureGround);

  timeLastBmeRead = millis();
}

void logSensorData()
{
  Serial1.print(millis());s(); // hardware time
  printGpsDate();s(); // utc time
  Serial1.print(bmeAltitude);s(); // altitude [bme]
  Serial1.print(gps.altitude.meters());s(); // altitude [gps]
  Serial1.print(gps.location.lat());s(); // latitude 
  Serial1.print(gps.location.lng());s(); // longitude
  Serial1.print(gps.satellites.value());s();
  Serial1.print(bmePressure);s(); // pressure (hPa)
  Serial1.print(bmeHumidity);s(); // humidity
  Serial1.print(bmeTemperature);s(); // temperature
  Serial1.print(acc[0]);s(); // acceleration (g) x
  Serial1.print(acc[1]);s(); // y
  Serial1.print(acc[2]);s(); // z
  Serial1.print(gyro[0]);s(); // gyroscope (deg/sec) x
  Serial1.print(gyro[1]);s(); // y
  Serial1.print(gyro[2]);s(); // z
  Serial1.print(gpsVelocity);s(); // gps velocity
  Serial1.print(pressureVelocity);s(); // pressure velocity
  Serial1.print(ventState);s(); // ventState
  Serial1.print(dropState);s(); // dropState
  Serial1.print(flightPhase);s(); // flightPhase
  Serial1.println();
  timeLastLog = millis();
}

void printGpsDate()
{
  Serial1.print(gps.date.month());
  Serial1.print(F("/"));
  Serial1.print(gps.date.day());
  Serial1.print(F("/"));
  Serial1.print(gps.date.year());

  Serial1.print(F(" "));

  if (gps.time.hour() < 10) Serial1.print(F("0"));
  Serial1.print(gps.time.hour());
  Serial1.print(F(":"));
  if (gps.time.minute() < 10) Serial1.print(F("0"));
  Serial1.print(gps.time.minute());
  Serial1.print(F(":"));
  if (gps.time.second() < 10) Serial1.print(F("0"));
  Serial1.print(gps.time.second());
  Serial1.print(F("."));
  if (gps.time.centisecond() < 10) Serial1.print(F("0"));
    Serial1.print(gps.time.centisecond());
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void updateMotors() {
  if(ventState == 1) {
    digitalWrite(VENT_MOTOR, HIGH);
  } else {
    digitalWrite(VENT_MOTOR, LOW);
  }

  if(dropState == 1) {
    digitalWrite(DROP_MOTOR, HIGH);
  } else {
    digitalWrite(DROP_MOTOR, LOW);
  }

  if(camState == 1) {
    digitalWrite(CAM_PIN, HIGH);
  } else {
    digitalWrite(CAM_PIN, LOW);
  }

  if(ledState == 1) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}
