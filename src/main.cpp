#include <Arduino.h>
#include <Wire.h> //I2C
#include <NewPing.h> //Ultrasonic Sensors
#include <LiquidCrystal_I2C.h> //LCD

/*#################################################################################################
####   Constants   ################################################################################
#################################################################################################*/
//general
const bool bLogging = 1; //Enable/Disable Logging to Serial
const long iSerial = 74880;
const int iLCDupdate=500; //How often should the LCD be refreshed
// Navigation limits
const int iCollision=40;
const int iObstacle=70;
//Motors
const int MLpwr = 2;
const int MLfwd = 4;
const int MLrev = 3;
const int MRpwr = 7;
const int MRfwd = 6;
const int MRrev = 5;
//Lights
const int iLightFrontLeft = 52;
const int iLightFrontRight = 50;
const int iLightBackLeft = 48;
const int iLightBackRight = 46;
//Sonics
const int iSonic = 4; //number of sensors
const int trigPinL = 23;
const int echoPinL = 23;
const int trigPinR = 25;
const int echoPinR = 25;
const int trigPinF = 27;
const int echoPinF = 27;
const int trigPinB = 29;
const int echoPinB = 29;

/*##################################################################################################
####   VARIABLES   ################################################################################
#################################################################################################*/
////Variables
//general
long cmdDuration = 0;
unsigned long cmdStarted = 0;
unsigned long lastLCDupdate=0;
//xxx char* cmdList[5] = {"", "", "", "", ""};
//xxx int cmdDurations[5] = {0, 0, 0, 0, 0};
//xxx int cmdNumber = 0;
//Sonics
int distL = 0;
int distR = 0;
int distF = 0;
int distB = 0;
NewPing sonar[iSonic] = {
    NewPing(trigPinL, echoPinL),
    NewPing(trigPinR, echoPinR),
    NewPing(trigPinF, echoPinF),
    NewPing(trigPinB, echoPinB)
};
//Display
LiquidCrystal_I2C lcd(0x27,16,2);
// Motors
int iSpeedL=0;
int iSpeedR=0;
/*#################################################################################################
####   Functions   ################################################################################
#################################################################################################*/
// Helper Functions ###############################################################################
void logger(char* info) {
  if (bLogging==1) {
    Serial.print(info);
  }
}
void loggerln(char* info) {
  if (bLogging==1) {
    Serial.println(info);
  }
}
void i2cScanner() {
  byte error, address;
  int nDevices;

  Serial.println("\nI2C Scanner");
  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 }
//xxx void newTask(char* cmd, int duration) {
//xxx   if (cmdNumber < 5) {
//xxx     cmdList[cmdNumber] = cmd;
//xxx     cmdDurations[cmdNumber] = duration;
//xxx     cmdNumber++;
//xxx   }
//xxx   else {
//xxx     //error
//xxx     Serial.println("Command cannot be added - cmd memory full");
//xxx   }
//xxx }
//xxx void finishTask() {
//xxx   for (int i; i<4; i++) {
//xxx     cmdList[i] = cmdList[i+1];
//xxx     cmdDurations[i] = cmdDurations[i+1];
//xxx   }
//xxx   cmdList[5]="";
//xxx   cmdDurations[4]=0;
//xxx }
// Light Functions ################################################################################
void Light(bool FL, bool FR, bool BL, bool BR) {
  digitalWrite(iLightFrontLeft, FL);
  digitalWrite(iLightFrontRight, FR);
  digitalWrite(iLightBackLeft, BL);
  digitalWrite(iLightBackRight, BR);
}
void Blink(bool FL, bool FR, bool BL, bool BR, int times) {
  for (int i=0; i<times; i++) {
    digitalWrite(iLightFrontLeft, FL);
    digitalWrite(iLightFrontRight, FR);
    digitalWrite(iLightBackLeft, BL);
    digitalWrite(iLightBackRight, BR);
    delay(250);
    digitalWrite(iLightFrontLeft, LOW);
    digitalWrite(iLightFrontRight, LOW);
    digitalWrite(iLightBackLeft, LOW);
    digitalWrite(iLightBackRight, LOW);
    delay(250);
  }
}
void readDist() {
  distL=sonar[0].ping_cm();
  distR=sonar[1].ping_cm();
  distF=sonar[2].ping_cm();
  distB=sonar[3].ping_cm();
  if (distL == 0) {
    distL = 10000;
  }
  if (distR == 0) {
    distR = 10000;
  }
  if (distF == 0) {
    distF = 10000;
  }
  if (distB == 0) {
    distB = 10000;
  }
}

// Display Functions #############################################################################
void printWelcome() {
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.backlight();
  lcd.print("Robot ready for");
  lcd.setCursor(0,1);
  lcd.print(" some ACTION!!!");
}
void printMeasures() {
  lcd.clear();
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.backlight();
  lcd.print(distL);
  lcd.setCursor(6, 0);
  lcd.print(distF);
  lcd.setCursor(12, 0);
  lcd.print(distR);
  lcd.setCursor(0, 1);
  lcd.print(iSpeedL);
  lcd.print("%");
  lcd.setCursor(6,1);
  lcd.print(distB);
  lcd.setCursor(11, 1);
  lcd.print(iSpeedL);
  lcd.print("%");
}

// Motor Functions ################################################################################
void dStop() {
  Light(0, 0, 1, 1);
  Serial.println("Stopping");
  digitalWrite(MLpwr, LOW);
  digitalWrite(MRpwr, LOW);
  iSpeedL=0;
  iSpeedR=0;
}
void dFWD(int speed) {
  Light(1, 1, 0, 0);
  Serial.print("Forward ");
  Serial.println(speed);
  digitalWrite(MLfwd, HIGH);
  digitalWrite(MLrev, LOW);
  digitalWrite(MRfwd, HIGH);
  digitalWrite(MRrev, LOW);
  analogWrite(MLpwr, speed);
  analogWrite(MRpwr, speed);
  iSpeedL=speed*100/255;
  iSpeedR=speed*100/255;
}
void dREV(int speed) {
  Light(0, 0, 1, 1);
  Serial.print("Reverse ");
  Serial.println(speed);
  digitalWrite(MLfwd, LOW);
  digitalWrite(MLrev, HIGH);
  digitalWrite(MRfwd, LOW);
  digitalWrite(MRrev, HIGH);
  analogWrite(MLpwr, speed);
  analogWrite(MRpwr, speed);
  iSpeedL=speed*(-100)/255;
  iSpeedR=speed*(-100)/255;
}
void dLEFT(int speed, int perc) {
  Serial.print("Left ");
  Serial.print(speed);
  Serial.print(" ");
  Serial.println(perc);
  digitalWrite(MLfwd, HIGH);
  digitalWrite(MLrev, LOW);
  digitalWrite(MRfwd, HIGH);
  digitalWrite(MRrev, LOW);
  analogWrite(MLpwr, speed * perc / 100);
  analogWrite(MRpwr, speed);
  iSpeedL=speed*perc/255;
  iSpeedR=speed*100/255;
  Light(0, 0, 0, 0);
  Blink(1, 0, 1, 0, 3);
  Light(1, 1, 0, 0);
}
void dRIGHT(int speed, int perc) {
  Serial.print("Right ");
  Serial.print(speed);
  Serial.print(" ");
  Serial.println(perc);
  digitalWrite(MLfwd, HIGH);
  digitalWrite(MLrev, LOW);
  digitalWrite(MRfwd, HIGH);
  digitalWrite(MRrev, LOW);
  analogWrite(MLpwr, speed);
  analogWrite(MRpwr, speed * perc / 100);
  iSpeedL=speed*100/255;
  iSpeedR=speed*perc/255;
  Light(0, 0, 0, 0);
  Blink(0, 1, 0, 1, 3);
  Light(1, 1, 0, 0);
}
void dTURN(char dir, int speed) {
  if (dir=='L') {
    Serial.println("Turn left");
    digitalWrite(MLfwd, HIGH);
    digitalWrite(MLrev, LOW);
    digitalWrite(MRfwd, LOW);
    digitalWrite(MRrev, HIGH);
    analogWrite(MLpwr, speed);
    analogWrite(MRpwr, speed);
    iSpeedL=speed*100/255;
    iSpeedR=speed*(-100)/255;
    Blink(1, 0, 1, 0, 3);
    Light(1, 0, 1, 0);
  }
  else if (dir=='R') {
    Serial.println("Turn right");
    digitalWrite(MLfwd, LOW);
    digitalWrite(MLrev, HIGH);
    digitalWrite(MRfwd, HIGH);
    digitalWrite(MRrev, LOW);
    analogWrite(MLpwr, speed);
    analogWrite(MRpwr, speed);
    iSpeedL=speed*(-100)/255;
    iSpeedR=speed*100/255;
    Blink(0, 1, 0, 1, 3);
    Light(0, 1, 0, 1);
  }
  else {
    Serial.println("Wrong turn command");
  }
}

// Main Functions #################################################################################
void setup() {
  Serial.begin(iSerial);
  //while (!Serial);
  //Display Setup
  Wire.begin();
  //i2cScanner();
  lcd.init();
  printWelcome();
  //Motor Setup
  pinMode(MLpwr,OUTPUT);
  pinMode(MLfwd,OUTPUT);
  pinMode(MLrev,OUTPUT);
  pinMode(MRpwr,OUTPUT);
  pinMode(MRfwd,OUTPUT);
  pinMode(MRrev,OUTPUT);
  dStop();
  //Light Setup
  pinMode(iLightFrontLeft, OUTPUT);
  pinMode(iLightFrontRight, OUTPUT);
  pinMode(iLightBackLeft, OUTPUT);
  pinMode(iLightBackRight, OUTPUT);
  Blink(1, 1, 1, 1, 5);

  //Setup Completed
  delay(2000);
}

void loop() {
  readDist();
  Serial.print("Left: ");
  Serial.print(distL);
  Serial.print("cm");
  Serial.print(" Right: ");
  Serial.print(distR);
  Serial.print("cm");
  Serial.print(" Front: ");
  Serial.print(distF);
  Serial.print("cm");
  Serial.print(" Back: ");
  Serial.print(distB);
  Serial.println("cm");
  //Update LCD Display each update intervall
  if (lastLCDupdate + iLCDupdate < millis()) {
      printMeasures();
      lastLCDupdate=millis();
  }

  /*
  dFWD: distF > iCollision && distL > iObstacle && distR > iObstacle
  dLEFT: distF > iCollision && distL > iObstacle && distR < iObstacle
  dRIGHT: distF > iCollision && distL < iObstacle && distR > iObstacle
  dTURN L: (distF < iCollision || distL < iCollision || distR < iCollision) && distL > distR
  dTURN R: (distF < iCollision || distL < iCollision || distR < iCollision) && distL <= distR
  */

  //If last command has no duration => drive
  if (!cmdDuration) {
    //if No Obstacle in Front => dFWD
    if (distF > iCollision && distL > iCollision && distR > iCollision) {
      if (distL > iObstacle && distR > iObstacle) {
        dFWD(255);
        cmdDuration=0;
        cmdStarted=millis();
      }
      else if (distL > iObstacle && distR < iObstacle) {
        dLEFT(255, distR*100/iObstacle);
        cmdDuration=0;
        cmdStarted=millis();
      }
      else if (distL < iObstacle && distR > iObstacle) {
        dRIGHT(255, distL*100/iObstacle);
        cmdDuration=0;
        cmdStarted=millis();
      }
    }
    else {
      //if distance left > Right => turn left
      dStop();
      if (distL > distR) {
        dREV(128);
        printMeasures();
        delay(500);
        dTURN('L', 128);
        cmdDuration=1500;
      }
      else {
        dREV(128);
        printMeasures();
        delay(500);
        dTURN('R', 128);
        cmdDuration=1500;
      }
    }
  }
  // if the last command has a duration wait until its done
  else {
    //when duration is reached => clear variables
    if (cmdStarted + cmdDuration < millis())
    {
      cmdDuration = 0;
      cmdStarted = millis();
      dStop();
    }
  }
  delay(10);
}
