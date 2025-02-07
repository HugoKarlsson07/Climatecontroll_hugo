/*
* Name: klimatekontroll
* Author: Hugo Karlsson
* Date: 2025-02-07
* Description: This project uses a adafruit SGP30 mox sensor for reading TVOC eCO2, , a analog temperature senso in C and a BME280 sensor that messures hpa that it displays on a 1306 OLED display
* It also uses a potentometer to control what sensor that is displayed on the oled display.
* Their is also a sparkfun SDcard reader that it writes all the data from the sensors to.
*/

// include librarys
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <BME280I2C.h>
#include <U8g2lib.h>

//termometer
const int termometer = A1;

// BME sensor 
BME280I2C bme;
struct SensorData {
  float temperature;
  float pressure;
  float humidity;
};
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

// SD card settings
const int chipSelect = 10;

// MOX sensor
Adafruit_SGP30 sgp;

// OLED display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
#define VALUES 128
int curve[VALUES];     // Graph data
int currentGraph = 0;  // Current graph to display (0: temperature, 1: MOX TVOC, 2: MOX eCO2, 3: pressure)

// Pin for potentiometer
const int potentiometerPin = A0;

void setup() {
  Serial.begin(115200);
  pinMode(termometer, INPUT); 
  u8g2.setFont(u8g2_font_ncenB08_tr); // Sätt en läsbar font

  while (!Serial) {
    ;  // Wait for the Serial port to initialize
  }

  pinMode(10, OUTPUT);  // Ensure SPI bus is correctly initialized

  delay(500);  // Give SD card reader time to start

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card ready.");

  // Initialize SGP30 sensor
  if (!sgp.begin()) {
    Serial.println("SGP30 sensor not found :(");
    while (1);
  }

  // Initialize OLED
  u8g2.begin();

  // Initialize BMP280 sensor
  readCalibrationData();  // Read BMP280 calibration data
}

void loop() {
  // Read sensor data
  SensorData bmpData = readBMP280();  // Get BMP280 data
  String moxData = readMOX();  // Get MOX sensor data

  // Prepare log data
  if (getttemp() > 100){
    writeToSDCard("Brinner");
    Serial.println(det verkar brinna/väldigt varmt);
  }

  String logData = "Temperature: " + String(gettemp()) + " C, Pressure: " + String(bmpData.pressure) + " hPa, " + moxData;

  // Check free space and write data to SD card
  if (checkFreeSpace(512)) {
    writeToSDCard(logData);
    Serial.println(logData);
  }


  handlePotentiometer();

  // Display graph based on selected mode
  if (currentGraph == 1) {
    oledGraph(gettemp(), 10, 40, "temp");  // Display temperature graph
  }
   if (currentGraph == 2) {
    oledGraph(sgp.TVOC, 0, 1000, "TVOC");  // Display MOX TVOC graph
  }  
  if (currentGraph == 3) {
    oledGraph(sgp.eCO2, 200, 1000, "CO2");  // Display MOX eCO2 graph
  }  
  if (currentGraph == 4) {
    oledGraph(bmpData.pressure, 900, 1100, "press");  // Display pressure graph
  }

  delay(500);  //delay for stability
}

/*
* Function: readMOX()
* Reads data from the MOX sensor.
* Parameters: None
* Returns: String - formatted MOX sensor readings.
*/
String readMOX() {
  if (!sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return "Error";
  }
  return "TVOC: " + String(sgp.TVOC) + " ppb, eCO2: " + String(sgp.eCO2) + " ppm";
}

/*
* Function: handlePotentiometer()
* Reads the potentiometer and updates the current graph selection.
* Parameters: None
* Returns: void
*/
void handlePotentiometer() {
  int potValue = analogRead(potentiometerPin);
  currentGraph = constrain(map(potValue, 0, 1023, 0, 3), 0, 4);  // Map potentiometer to graph index
}

 /*
* Function: oledGraph()
* Displays sensor data as a graph on the OLED display.
* Parameters: float value, float minValue, float maxValue, String text
* Returns: void
*/
void oledGraph(float value, float minValue, float maxValue, String text){
  for (int i = 1; i < VALUES; i++) {
    curve[i - 1] = curve[i];
  }
  curve[VALUES - 1] = map(constrain(value, minValue, maxValue), minValue, maxValue, 63, 0);

  u8g2.clearBuffer();
  for (int i = 0; i < VALUES - 1; i++) {
    u8g2.drawPixel(i, curve[i]);
    u8g2.drawStr(90, 60, text.c_str());
  }
  u8g2.sendBuffer();
}

// Function to read BMP280 sensor data

/*
* Function: readBMP280()
* Reads data from BME 280 sensor
* Parameters: None
* Returns: String data that contains pressure readings
*/
SensorData readBMP280() {
  SensorData data;
  // Request pressure and temperature data
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);  // Start address for pressure and temperature readings
  Wire.endTransmission();

  Wire.requestFrom(0x76, 6);
  while (Wire.available() < 6);

  uint8_t msbP = Wire.read();
  uint8_t lsbP = Wire.read();
  uint8_t xlsbP = Wire.read();
  uint8_t msbT = Wire.read();
  uint8_t lsbT = Wire.read();
  uint8_t xlsbT = Wire.read();

  // Combine raw data
  int32_t adc_P = ((uint32_t)msbP << 12) | ((uint32_t)lsbP << 4) | (xlsbP >> 4);

  // Calculate temperature and pressure
  data.pressure = compensatePressure(adc_P) / 100.0;  // Pressure in hPa
  
  return data;
}
/*
* Function: gettemp()
* Reads the analog value from the temperature sensor, calculates the temperature in Celsius,
* and prints it to the serial monitor.
* Parameters: None
* Returns: float - Temperature in Celsius
*/
float gettemp() {
  int Vo;
  float R1 = 10000;  // value of R1 on board
  float logR2, R2, T;
  float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741;

  Vo = analogRead(termometer);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);  //calculate resistance on thermistor
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));  // temperature in Kelvin
  T = T - 273.15;                                              //convert Kelvin to Celcius
  Serial.print("Temperature: ");
  Serial.print(T);
  Serial.println(" C");
  return T;
}
/*
* Function: writeToSDCard() 
* writes to the text file on the sd card
* Parameters: const String& data
* Returns: non
*/
void writeToSDCard(const String& data) {
  delay(10);
  File file = SD.open("log.txt", FILE_WRITE);  // Open or create log.txt
  if (file) {
    file.println(data);  // Write data to the file
    Serial.println("Data written to log.txt.");
    file.close();
  } else {
    Serial.println("Error: Could not open log.txt for writing.");
  }
}

/*
* Function: checkFreeSpace() 
* checks if their is space on the sd card
* Parameters: size_t requiredBytes
* Returns: true if their is free space on the sd card
*/
bool checkFreeSpace(size_t requiredBytes) {
  return true;  // Assuming sufficient space for now
}



/*
* Function: readCalibrationData()
* Calibration data from I2C communikation
* Parameters: None
* Returns: none
*/
void readCalibrationData() {
  Wire.beginTransmission(0x76);
  Wire.write(0x88);  // Start address for calibration data
  Wire.endTransmission();
  
  Wire.requestFrom(0x76, 24);
  dig_T1 = (Wire.read() | (Wire.read() << 8));
  dig_T2 = (Wire.read() | (Wire.read() << 8));
  dig_T3 = (Wire.read() | (Wire.read() << 8));
  dig_P1 = (Wire.read() | (Wire.read() << 8));
  dig_P2 = (Wire.read() | (Wire.read() << 8));
  dig_P3 = (Wire.read() | (Wire.read() << 8));
  dig_P4 = (Wire.read() | (Wire.read() << 8));
  dig_P5 = (Wire.read() | (Wire.read() << 8));
  dig_P6 = (Wire.read() | (Wire.read() << 8));
  dig_P7 = (Wire.read() | (Wire.read() << 8));
  dig_P8 = (Wire.read() | (Wire.read() << 8));
  dig_P9 = (Wire.read() | (Wire.read() << 8));
}


// Pressure compensation for BMP280

/*
* Function: compensatePressure()
* calibrates the pressure raw data 
* Parameters: int32_t adc_P
* Returns: float calibrated pessure value
*/
float compensatePressure(int32_t adc_P) {
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

  if (var1 == 0) {
    return 0;  // Prevent division by zero
  }

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)p / 200;  // Return pressure in Pascal
}
