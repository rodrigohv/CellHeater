//includes for temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>


//includes for lcd display
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//include for PID-library
#include <PID_v1.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Potentiometer pin
#define analogPin A3

//Output pin for heat power
#define outputPin 3

//Temperature resolution
#define TEMPRES 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress Thermometer[5];

//Adress of the controling themometer
byte ControlAdress[8]={40,51,223,57,3,0,0,164};
int ControlDevice;

// Variable to hold number of devices found
int numDev;

// LCD init
LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//Vars for temp
double targetTemp = 37.0;
double currentTemp = 20.0;
double Temp[5];
double output = 0;
char tempChar[8];

//Vars for PID
double Kp = 2.5;
double Ki = 1.2;
double Kd = 0.1;

//Specify the links and initial tuning parameters
PID myPID(&currentTemp, &output, &targetTemp, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialise serial over USB
  Serial.begin(115200);

  // initialize the lcd
  lcd.begin(16, 2);
  
  lcd.print("Locating devices.");
  Serial.println("Locating devices.");
  
  //initialise the temp sensors on onewWire bus
  sensors.begin();


  // print number of sensors
  numDev=sensors.getDeviceCount();
  
  lcd.setCursor(0, 0);
  lcd.print("Found ");
  lcd.print(numDev);
  lcd.print(" devices");
  Serial.print("Found ");
  Serial.print(numDev);
  Serial.println(" devices");

  //assign temp sensor adress and print it
  for (int i=0;i<numDev;i++) {
    if (!sensors.getAddress(Thermometer[i], i)) {
      lcd.setCursor(0, 1);
      lcd.print("Error: device ");
      lcd.print(i);
      Serial.print("Error: device ");
      Serial.println(i);
      delay(1000);
    } else {
      lcd.clear();
      lcd.print("Device ");
      lcd.print(i);
      lcd.setCursor(0, 1);
      Serial.print("Device ");
      Serial.print(i);
      Serial.print(": ");
      printAddress(Thermometer[i]);
      sensors.setResolution(Thermometer[i], TEMPRES);
      delay(1000);

      //Check if this thermometer is the controlling one
      if (ByteArrayCompare(Thermometer[i],ControlAdress,8)) {ControlDevice=i;}
    }
  }

  lcd.clear();
  lcd.print("Control them is:");
  lcd.setCursor(0,1);
  lcd.print(ControlDevice);
  Serial.print("Control themometer is: ");
  Serial.println(ControlDevice);
  lcd.clear();

  //initialise PID library
  myPID.SetMode(AUTOMATIC);

  //PID set up with potentiometer
  lcd.print("For PID setup");
  lcd.setCursor(0, 1);
  lcd.print("turn pot down.");
  if (countDownLow(5)) {
    lcd.clear();
    lcd.print("Now turn up.");


    //adjust ranges and resolution of pid parameter setting
    //countDownValue(m,n,s) [m=maximal value, n=total number of steps, s=time in seconds]
    if (countDownHigh(5)) {
      lcd.clear();
      lcd.print("Set P");
      Kp = countDownValue(10,100,10);

      lcd.clear();
      lcd.print("Set I");
      Ki = countDownValue(10,100,10);

      lcd.clear();
      lcd.print("Set D");
      Kd = countDownValue(10,100,10);
    }
  }
  lcd.clear();
  lcd.print("PID set to:");
  lcd.setCursor(0, 1);
  lcd.print(Kp);
  lcd.print(" ");
  lcd.print(Ki);
  lcd.print(" ");
  lcd.print(Kd);
  
  Serial.print("PID set to:");
  Serial.print(" ");
  Serial.print(Kp);
  Serial.print(" ");
  Serial.print(Ki);
  Serial.print(" ");
  Serial.println(Kd);

  myPID.SetTunings(Kp, Ki, Kd);

  delay(1000);
  lcd.clear();

  Serial.print("Zero time: ");
  Serial.println(millis());
}

void loop() {

  // Request
  sensors.requestTemperatures();

  delay(1200);

  for (int i=0;i<numDev;i++) {
    //Read and debounce
    double oldTemp = Temp[i];
    Temp[i] = sensors.getTempC(Thermometer[i]);
    if (Temp[i] == -127.0) {Temp[i] = oldTemp;}
    if (Temp[i] == 0.0) {Temp[i] = oldTemp;}
    if (Temp[i] == 85.0) {Temp[i] = oldTemp;}
    if (i==ControlDevice) {currentTemp=Temp[i];}
    
  }
  
  // Read targetTemp
  targetTemp = float(int(map(analogRead(analogPin), 0, 1023, 100, 160))) / 4;

  //PID recalculate
  myPID.Compute();

  //Set output to new value
  analogWrite(outputPin, int(output));

  //Draw it all
  updateScreen();

}

void updateScreen() {
  lcd.home();
  lcd.print("Curr:");
  lcd.print(currentTemp);
  lcd.print("  ");

  lcd.setCursor(0, 1);
  lcd.print("Targ:");
  lcd.print(targetTemp);

  lcd.print(" P:");
  lcd.print(int(output));
  lcd.print("  ");
}

void loop1() {
  delay(10000);
  Serial.print("Time: ");
  Serial.print(millis());

  for (int i=0;i<numDev;i++) {
    Serial.print("  Temp");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(Temp[i]);
  }
  Serial.print("  Target: ");
  Serial.print(targetTemp);
  Serial.print("  Output: ");
  Serial.println(int(output));
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) {
       lcd.print("0");
       Serial.print("0");
    }
    lcd.print(deviceAddress[i], HEX);
    Serial.print(deviceAddress[i], HEX);
  }
  Serial.println();
}

//check if pot is turned all up
boolean countDownLow(int s) {
  unsigned long tid = millis();
  while (millis() <= tid + s * 1000) {
    lcd.setCursor(14, 0);
    if (analogRead(analogPin) < 10) {
      lcd.print("Y");
    } else {
      lcd.print("N");
    }
    lcd.print(" ");
  }
  if (analogRead(analogPin) < 10) {
    return true;
  } else {
    return false;
  }
}
//check if pot  is turned all down
boolean countDownHigh(int s) {
  unsigned long tid = millis();
  while (millis() <= tid + s * 1000) {
    lcd.setCursor(14, 0);
    if (analogRead(analogPin) > 1013) {
      lcd.print("Y");
    } else {
      lcd.print("N");
    }
    lcd.print(" ");
  }
  if (analogRead(analogPin) > 1013) {
    return true;
  } else {
    return false;
  }
}

// m=maximal value, n=steps , s=time to set
double countDownValue(double m, double n, int s) {
  unsigned long tid = millis();
  while (millis() <= tid + s * 1000) {
    lcd.setCursor(0, 1);
    lcd.print(map(analogRead(analogPin), 0, 1023, 0, n)/n*m);
    lcd.print("  ");
  }
  return map(analogRead(analogPin), 0, 1023, 0, n)/n*m;
}

boolean ByteArrayCompare(byte a[], byte b[], int array_size)
{
   for (int i = 0; i < array_size; ++i)
     if (a[i] != b[i])
       return(false);
   return(true);
}

