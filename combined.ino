#include <SPI.h>



#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

//m701   Fuel Pres         A0
//702   Boost Pres        A1
//703   Block Temp        A2
//704   h2o Pres          A3
//705   h20 temp          A4
//706   oil Pres          a5
//707   oil temp          a6
//708   wbo2              A7
//709   AFR               (calculated)



const int FuelpressurePin = A0;  //Define analog input pin for pressure sensor
const int boostpressurePin = A1; // Define analog input pin for pressure sensor
const int THERMISTOR_PIN = A2;
const int CoolantpressurePin = A3; // Define analog input pin for pressure sensor
const int CoolantTHERMISTOR_PIN = A4;
const int OilpressurePin = A5; // Define analog input pin for pressure sensor
const int OilTHERMISTOR_PIN = A6;
const int O2_PIN = A7;


const float referenceVoltage = 5;//  Define reference voltage
const double MAX_ADC            = 1024.0;   //Max number of ADC steps (10-bit in this case)
const int NUM_READINGS = 5;  //Number of readings to average


const float AFRoutputMin = 0.68;
const float AFRoutputMax = 1.36;


const float FuelminVoltage = 0.5;//  Define minimum voltage value
const float FuelmaxVoltage = 4.5; // Define maximum voltage value
const float FuelminPressure = 0; // Define minimum pressure value
const float FuelmaxPressure = 1000;  //Define maximum pressure value


const float CoolantminVoltage = 0.5; // Define minimum voltage value
const float CoolantmaxVoltage = 4.5; // Define maximum voltage value
const float CoolantminPressure = 0; // Define minimum pressure value
const float CoolantmaxPressure = 1000;  //Define maximum pressure value

const float OilminVoltage = 0.5; // Define minimum voltage value
const float OilmaxVoltage = 4.5; // Define maximum voltage value
const float OilminPressure = 0; // Define minimum pressure value
const float OilmaxPressure = 1000; // Define maximum pressure value

const float boostminVoltage = 0.4; // Define minimum voltage value
const float boostmaxVoltage = 4.65;  //Define maximum voltage value
const float boostminPressure = 20;  //Define minimum pressure value
const float boostmaxPressure = 300; // Define maximum pressure value

const double BALANCE_RESISTOR   = 2000.0;   //Measured value of on-board divider resistor
const double BETA               = 3600.0;   // Beta value (from datasheet)
const double ROOM_TEMP          = 298.15;   // room ambient temperature in Kelvin
const double RESISTOR_ROOM_TEMP = 10000.0;  // Measured value of thermistor at room temp
double currentTemperature = 0;              // Variable to hold measured temperature

const double CoolantBALANCE_RESISTOR   = 1000.0; //  Measured value of on-board divider resistor
const double CoolantBETA               = 3446;   // Beta value (from datasheet)
const double CoolantROOM_TEMP          = 293.15;  //  room ambient temperature in Kelvin
const double CoolantRESISTOR_ROOM_TEMP = 2480.0;  // Measured value of thermistor at room temp
double CoolantcurrentTemperature = 0;  

const double OilBALANCE_RESISTOR   = 1000.0;   //Measured value of on-board divider resistor
const double OilBETA               = 3446;   // Beta value (from datasheet)
const double OilROOM_TEMP          = 293.15; //   room ambient temperature in Kelvin
const double OilRESISTOR_ROOM_TEMP = 2480.0; //  Measured value of thermistor at room temp
double OilcurrentTemperature = 0;


float fuelPressureReadings[NUM_READINGS];
float CoolantPressureReadings[NUM_READINGS];
float OilPressureReadings[NUM_READINGS];
float boostPressureReadings[NUM_READINGS];
double temperatureReadings[NUM_READINGS];
double CoolanttemperatureReadings[NUM_READINGS];
double OiltemperatureReadings[NUM_READINGS];
double AFRReadings[NUM_READINGS];
int readingIndex = 0;
float myArray[9];



unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
    SERIAL_PORT_MONITOR.begin(9600);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
}






void loop() {
    

fuelPressureReadings[readingIndex] = convertToPressure(analogRead(FuelpressurePin), FuelminVoltage, FuelmaxVoltage, FuelminPressure, FuelmaxPressure, referenceVoltage);
CoolantPressureReadings[readingIndex] = convertToPressure(analogRead(CoolantpressurePin), CoolantminVoltage, CoolantmaxVoltage, CoolantminPressure, CoolantmaxPressure, referenceVoltage);
OilPressureReadings[readingIndex] = convertToPressure(analogRead(OilpressurePin), OilminVoltage, OilmaxVoltage, OilminPressure, OilmaxPressure, referenceVoltage);
boostPressureReadings[readingIndex] = convertToPSI(convertToPressure(analogRead(boostpressurePin), boostminVoltage, boostmaxVoltage, boostminPressure, boostmaxPressure, referenceVoltage));
temperatureReadings[readingIndex] = readThermistor();
AFRReadings[readingIndex] = readAFR();
CoolanttemperatureReadings[readingIndex] = DreadThermistor(CoolantTHERMISTOR_PIN,CoolantBALANCE_RESISTOR,CoolantRESISTOR_ROOM_TEMP,CoolantBETA);
OiltemperatureReadings[readingIndex] = DreadThermistor(OilTHERMISTOR_PIN,OilBALANCE_RESISTOR,OilRESISTOR_ROOM_TEMP,OilBETA);
readingIndex++;


if (readingIndex >= NUM_READINGS) {
  readingIndex = 0;
}

float fuelPressureSum = 0;
float AFRSum = 0;
float CoolantPressureSum = 0;
float OilPressureSum = 0;
float boostPressureSum = 0;
double temperatureSum = 0;
double CoolanttemperatureSum = 0;
double OiltemperatureSum = 0;


for (int i = 0; i < NUM_READINGS; i++) {
  fuelPressureSum += fuelPressureReadings[i];
  AFRSum += AFRReadings[i];
  CoolantPressureSum += CoolantPressureReadings[i];
  OilPressureSum += OilPressureReadings[i];
  boostPressureSum += boostPressureReadings[i];
  temperatureSum += temperatureReadings[i];
  CoolanttemperatureSum += CoolanttemperatureReadings[i];
  OiltemperatureSum += OiltemperatureReadings[i];
}



float fuelPressureAvg = fuelPressureSum / NUM_READINGS;
float CoolantPressureAvg = CoolantPressureSum / NUM_READINGS;
float OilPressureAvg = OilPressureSum / NUM_READINGS;
float boostPressureAvg = boostPressureSum / NUM_READINGS;
float temperatureAvg = temperatureSum / NUM_READINGS;
float CoolanttemperatureAvg = CoolanttemperatureSum / NUM_READINGS;
float OiltemperatureAvg = OiltemperatureSum / NUM_READINGS;
float AFRAvg = AFRSum / NUM_READINGS;
float AFRRATIO = AFRAvg * 14.7;


Serial.print("Block Temp: ");
Serial.print(temperatureAvg);
Serial.print("C |  Coolant Temp:  ");
Serial.print(CoolanttemperatureAvg);
Serial.print("C |***Oil Temp:  ");
Serial.print(OiltemperatureAvg);
Serial.print("C***|  ");
Serial.print("Fuel Pressure: ");
Serial.print(fuelPressureAvg);
Serial.print(" kPa |  Coolant:  ");
Serial.print(CoolantPressureAvg);
Serial.print(" kPa |***Oil Pres:  ");
Serial.print(OilPressureAvg);
Serial.print(" kPa***|  ");
Serial.print("Boost: ");
Serial.print(boostPressureAvg);
Serial.print(" PSI   |  ");
Serial.print("AFR: ");
Serial.print(AFRAvg);
Serial.print(" Lamba   |  ");
Serial.print("AFR: ");
Serial.println(AFRRATIO);


myArray[0]=fuelPressureAvg;
myArray[1]=boostPressureAvg;
myArray[2]=temperatureAvg;
myArray[3]=CoolantPressureAvg;
myArray[4]=CoolanttemperatureAvg;
myArray[5]=OilPressureAvg;
myArray[6]=OiltemperatureAvg;
myArray[7]=AFRAvg;
myArray[8]=AFRRATIO;


//701   Fuel Pres         A0
//702   Boost Pres        A1
//703   Block Temp        A2
//704   h2o Pres          A3
//705   h20 temp          A4
//706   oil Pres          a5
//707   oil temp          a6
//708   wbo2              A7
//709   AFR               (calculated)


 ////  Set up the initial CAN ID
//  int canId = 0x701;
int canId = 701;

for (int i = 0; i < 9; i++) {

float floatValue = myArray[i];
unsigned char canMessageBytes[8];
unsigned char* floatBytes = (unsigned char*)&floatValue;

for (int i = 0; i < 4; i++) {
  canMessageBytes[i + 0] = floatBytes[i];
}


    

CAN.MCP_CAN::sendMsgBuf(canId, 0, 8, canMessageBytes);


 //    Increment the CAN ID for the next variable
    canId++;
}

      delay(5000);
  }
  






//===============================================================================
//Functions
//===============================================================================
 //Function to convert sensor value to pressure in kPa
float convertToPressure(int sensorValue, float minVoltage, float maxVoltage, float minPressure, float maxPressure, float referenceVoltage) {
  float voltage = (sensorValue / 1023.0) * referenceVoltage;
  float pressure = (voltage - minVoltage) / (maxVoltage - minVoltage) * (maxPressure - minPressure) + minPressure;
  return pressure;
}

// Function to convert pressure from kPa to PSI
float convertToPSI(float pressure) {
  float relativePressure = pressure - 101.3;
  float psi = relativePressure * 0.145038;
  return psi;
}

float readAFR(){
  float inputVoltage = analogRead(O2_PIN) * (5.0 / 1023.0);
  float outputValue = mapFloat(inputVoltage, 0.0, 5.0, AFRoutputMin, AFRoutputMax);
  return outputValue;
}


// Function to read the temperature from the thermistor
double readThermistor() {
 //  Local Variables
  double rThermistor = 0;        //     Thermistor resistance value
  double tKelvin     = 0;        //     Calculated temperature in Kelvin
  double tCelsius    = 0;        //     Calculated temperature in celsius
  int    adcSample   = 0;        //     ADC measurement

  adcSample = analogRead(THERMISTOR_PIN); //  read from pin and store
  rThermistor = BALANCE_RESISTOR * ( (MAX_ADC / adcSample) - 1);
  tKelvin = (BETA * ROOM_TEMP) / (BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));
  tCelsius = tKelvin - 273.15;   //convert kelvin to celsius 
  return tCelsius;   //  Return the temperature in Celsius
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

double DreadThermistor(int THERMISTOR_PIN,float BALANCE_RESISTOR,float RESISTOR_ROOM_TEMP,float BETA) {
///   Local Variables
  double rThermistor = 0;         //    Thermistor resistance value
  double tKelvin     = 0;         //    Calculated temperature in Kelvin
  double tCelsius    = 0;         //    Calculated temperature in celsius
  double adcSample   = 0;         //    ADC measurement

  adcSample = analogRead(THERMISTOR_PIN); //  read from pin and store

  adcSample =  MAX_ADC / adcSample - 1;
  adcSample = BALANCE_RESISTOR / adcSample;

  float steinhart;
  steinhart = adcSample / RESISTOR_ROOM_TEMP; //     (R/Ro)
  steinhart = log(steinhart);                 //  ln(R/Ro)
  steinhart /=BETA;                  //  1/B * ln(R/Ro)
  steinhart += 1.0 / (20 + 273.15);  //+ (1/To)
  steinhart = 1.0 / steinhart;                //  Invert
  steinhart -= 273.15;                         // convert absolute temp to C

  return steinhart;
}
