#include "ACS712.h" //  ACS712 library
#include <LiquidCrystal.h>  //  LCD module library & config
#include <SoftwareSerial.h> //  Software serial library for SIM800L communication on Arduino Uno
//  I/O Pins
#define motor  10  //  AC motor is connected here
#define b_switch 13  //  Switch to turn device on/off
#define lcdToggle 2 //  Button to switch display screen
//  Analog Pins
#define motorTempPin A1  //  Temperature sensor (motor temperature)
#define roomTempPin A2  //  Temperature sensor 1 (room temperature)
#define P1_Pin A3 //  Phase-1 analog voltage pin
#define P2_Pin A4 //  Phase-2 analog voltage pin
#define P3_Pin A5 //  Phase-3 analog voltage pin
//  Initialize all libraries
ACS712  ACS(A1, 5.0, 1023, 185);  //  20A used with 185mA sensitivity. Sensor is on Pin A1
LiquidCrystal lcd(3, 4, 5, 6, 7, 8);  //  LCD module pins on board - RS,EN,D4,D5,D6,D7
SoftwareSerial simModule(0, 1); //  Instantiate serial communication on software serial pins 0 & 1
//  Global variables
bool motorState;  //  Keeps track of motor state (on/off)
bool smsTurnOn = 1; //  If turn motor on command is received from SMS
byte tempState; //  Keeps track of temperature operatable state
byte voltageState;  //  Keeps track of voltage operatable state
double P1_current, P2_current, P3_current;  //  Current values for phase 1, 2 & 3
int roomTemp, motorTemp;  //  Room and motor temperature values
const int maxTemp = 60;  //  Maximum temperature of device
const int minTemp = -20;  //  Minimum temperature of device
int P1_voltage, P2_voltage, P3_voltage;  //  Voltage values for phase 1, 2 & 3
const int min_p_voltage = 180;  //  Minimum device operating voltage
const int max_p_voltage = 250;  //  Maximum device operating voltage

void setup() {
  pinMode(b_switch,  INPUT_PULLUP);
  pinMode(lcdToggle,  INPUT_PULLUP);
  pinMode(motor, OUTPUT);
  simModule.begin(9600);  //  Set baud rate and begin serial communication with SIM module
  ACS.autoMidPoint(); //  Configure ACS module
  lcd.begin(20, 4); lcd.clear();  //  Initialize LCD and clear screen
  lcd.print("Connecting SIM card");
  lcd.setCursor(4, 1);  lcd.print("Please wait");
  for (byte i = 15;  i > 0;  i--) { //  Display time countdown to ensure SIM connection
    lcd.setCursor(8, 3);
    if (i >= 10) lcd.print(i);
    else lcd.print('0' + i);
    delay(1000);
  }
  //  SIM module initialization commands
  simModule.println("AT");
  delay(100);
  simModule.println("AT+CMGDA=\"DEL ALL\"");
  delay(100);
  simModule.println("AT+CMGF=1");
  delay(100);
  simModule.println("AT+CMGR=1");  delay(100);
  simModule.println("AT+CNMI=1,2,0,0,0");
  delay(100);
  lcd.clear();
}

void loop() {
  if (simModule.available()) {  //  If SIM module gets a message
    smsData();  //  Processes message received and returns a reply
  }
  if (digitalRead(b_switch)) {   // Switch is off, device is on standby
    lcd.clear();
    motorState = 0;
    digitalWrite(motor, motorState);
    const char message[] = "SMS Based LCD 3-Phase Induction Motor Protection System";
    while (!digitalRead(b_switch)) { //  Runs infinitely as long as switch is off
      lcd.print("****Standby Mode****");
      for (byte i = 0; i < 56; i++) { //  Scrolling text on screen
        lcd.cursor();
        if (i > 19 && i < 38) lcd.setCursor(i - 20, 2);
        else if (i <= 19)  lcd.setCursor(0 + i, 1);
        else if (i >= 38) lcd.setCursor(i - 38, 3);
        lcd.print(message[i]);
        delay(100);
      }
      delay(300);
      lcd.clear();
    }
    lcd.clear();
    lcd.noCursor();
  } else {  //  Switch is toggled on, device is operating
    getData();  //  Obtain data from sensors
    lcdDisplay(); //  Renders data on LCD
    if (motorTemp < minTemp)  tempState = 0;  //  Low motor temperature
    else if (motorTemp > minTemp && motorTemp < maxTemp)   tempState = 1;  //  Normal motor temperature
    else if (motorTemp > maxTemp)  tempState = 2;  //  High motor temperature
    if (P1_voltage >= min_p_voltage && P2_voltage < min_p_voltage && P3_voltage < min_p_voltage) {  //  Device runs on a single phase
      singlePhase();
    } else if (P1_voltage < min_p_voltage && P2_voltage >= min_p_voltage && P3_voltage < min_p_voltage) {  //  Device runs on a single phase
      singlePhase();
    } else if (P1_voltage < min_p_voltage && P2_voltage < min_p_voltage && P3_voltage >= min_p_voltage) {  //  Device runs on a single phase
      singlePhase();
    } else if (P1_voltage >= min_p_voltage && P2_voltage >= min_p_voltage && P3_voltage < min_p_voltage) {  //  Device runs on double phases
      doublePhase();
    } else if (P1_voltage >= min_p_voltage && P2_voltage < min_p_voltage && P3_voltage >= min_p_voltage) {  //  Device runs on double phases
      doublePhase();
    } else if (P1_voltage < min_p_voltage && P2_voltage >= min_p_voltage && P3_voltage >= min_p_voltage) {  //  Device runs on double phases
      doublePhase();
    } else if (P1_voltage >= min_p_voltage && P2_voltage >= min_p_voltage && P3_voltage >= min_p_voltage) {  //  Device runs on all phases
      voltageState = 1; //  Voltage is normal
      lcd.setCursor(0, 0);
      if (tempState == 0) { //  Low temperature
        lcd.print("Norm VO, Low TP ");  //  Normal Voltage, Low Temperature
        lcd.print("-Off");  //  Device is off
        motorState = 0; //  Motor isn't running
      } else if (tempState == 2) {  //  High temperature
        lcd.print("Norm VO, High TP");
        lcd.print("-Off");  //  Device is off
        motorState = 0; //  Motor isn't running
      } else if (tempState == 1) {
        lcd.print("Norm VO, Norm TP");
        lcd.setCursor(16, 0);
        if (smsTurnOn) {  //  Device receives turn on command from SMS
          motorState = 1; //  Turn on motor
          lcd.print("- On");
        } else {  //  Device receives turn off command from SMS
          motorState = 0; //  Turn off motor
          lcd.print("-Off");
        }
      }
    }
  }
  digitalWrite(motor, motorState);  //  Apply state to motor
}

void smsData() {
  simModule.println("AT+CMGR=1");
  delay(100);
  String getMessage = simModule.readString(); //  Reads message content into a string
  getMessage.toUpperCase(); //  Convert the string to uppercase for effective comparison
  char senderNumber[15];  //  Obtains sender number from an SMS
  for (byte i = 0; i < 14; i++) {
    senderNumber[i] = getMessage.charAt(i + 9);  //  Store SMS sender's number into an array
  }
  //  The following lines of code configure a message to return to the sender based on device's state
  String message;  //  Holds the SMS content message to be replied
  if (digitalRead(b_switch)) {  //  If device has been turned off via switch
    message = "Device in standby mode, push the RED power button to turn on";
  } else {  //  If device is running i.e. not in standby mode
    if (getMessage.indexOf("TURN ON") > 0) {  //  If the SMS received is "Turn on"
      if (!voltageState || !tempState) {  //  If we have abnormal voltage or temperature
        smsTurnOn = 0;  //  Don't turn device on
        message = "Command received, motor turn-on unsuccessful, Check status!";
      } else if (voltageState && tempState) { //  If both voltage & temperature are okay for operation:
        smsTurnOn = 1;  //  Turn on device
        message = "Command received, motor turned on successfully";
      }
    } else if (getMessage.indexOf("TURN OFF") > 0) {  //  If the SMS received is "Turn off"
      smsTurnOn = 0;  //  Turn off device
      if (!motorState) {  //  If motor not running before receiving turn off command
        message = "Command received, motor not initially working, Check status!";
      }
      else {  //  If motor is running before receiving turn off command
        message = "Command received, motor turned off successfully";
      }
    } else if (getMessage.indexOf("STATUS") > 0) {  //  If the SMS received is "Status"
      String mStatus; //  Holds the current motor operating status (active/inactive)
      if (motorState) mStatus = "Active";
      else mStatus = "Inactive";
      //  Return a summary of device's operating state
      message = {"Motor is " + mStatus + ", *P1 (" + String(P1_voltage) + "V, " + String(P1_current) + "A), *P2 (" + String(P2_voltage) + "V, "
                 + String(P2_current) + "A), *P3 (" + String(P3_voltage) + "V, " + String(P3_current) + "A)"
                };
    } else if (getMessage.indexOf("COMMANDS") > 0) {  //  If the SMS received is "Commands"
      //  Returns a short list of possible commands
      message = "(1) Turn on (2) Turn off (3) Status";
    }
  }
  //  Return SMS message composed to sender
  simModule.println("AT+CMGF=1");
  delay(100);
  simModule.println("AT+CMGS=\"" + String(senderNumber) + "\"");
  delay(100);
  simModule.print(message);
  delay(100);
  simModule.write(26);
}
//  Obtains data from all sensors
void getData() {
  P1_voltage = (analogRead(P1_Pin) * 5.0 / 1023.0 * 100); //  Phase 1 analog voltage value
  P2_voltage = (analogRead(P2_Pin) * 5.0 / 1023.0 * 100); //  Phase 2 analog voltage value
  P3_voltage = (analogRead(P3_Pin) * 5.0 / 1023.0 * 100); //  Phase 3 analog voltage value
  P3_current = ACS.mA_AC() * 0.001;  //  Obtain current readings from ACS712 sensor and convert from mA to Ampere
  //  Based on the phase voltage, random current values are generated for phase 1 and phase 2
  //  You can replace them if you have actual sensors for both phases
  if (P1_voltage >= 100)  P1_current = random(0.1, 0.5);
  else P1_current = 0.0;
  if (P2_voltage >= 100)  P2_current = random(0.5, 2.5);
  else P2_current = 0.0;
  roomTemp = (analogRead(roomTempPin) * 5.0 / 1023.0 * 100);  //  Room temperature from room temperature sensor
  motorTemp = (analogRead(motorTempPin) * 5.0 / 1023.0 * 100);  //  Motor temperature from motor temperature sensor
}

void lcdDisplay() { //  Display all contents on screen
  lcd.setCursor(0, 1);  lcd.print("  |  P1 | P2  | P3  ");
  lcd.setCursor(0, 2);  lcd.print("V - " + String(P1_voltage) + 'V');
  lcd.setCursor(0, 3);  lcd.print("C - ");
  lcd.setCursor(8, 3);  lcd.write('|');
  lcd.setCursor(14, 3);  lcd.write('|');
  if (P1_voltage > 9 && P1_voltage <  100) {
    lcd.setCursor(7, 2);  lcd.print(" | ");
  } else if (P1_voltage <= 9) {
    lcd.setCursor(6, 2);  lcd.print("  | ");
  } else {
    lcd.setCursor(8, 2);  lcd.print("| ");
  }
  lcd.setCursor(10, 2);  lcd.print(String(P2_voltage) + 'V');
  if (P2_voltage > 9 && P2_voltage <  100) {
    lcd.setCursor(13, 2);  lcd.print(" | ");
  } else if (P2_voltage < 10) {
    lcd.setCursor(12, 2);  lcd.print("  | ");
  } else {
    lcd.setCursor(14, 2);  lcd.print("| ");
  }
  lcd.setCursor(16, 2);  lcd.print(String(P3_voltage) + 'V');
  if (P3_voltage > 9 && P3_voltage <  100) {
    lcd.setCursor(19, 2);  lcd.write(' ');
  } else if (P3_voltage < 10) {
    lcd.setCursor(18, 2);  lcd.print("  ");
  }
  lcd.setCursor(4, 3);  lcd.print(P1_current);
  lcd.setCursor(7, 3);  lcd.write('A');
  lcd.setCursor(10, 3);  lcd.print(P2_current);
  lcd.setCursor(13, 3);  lcd.write('A');
  lcd.setCursor(15, 3);  lcd.print(P3_current, 2);
  lcd.setCursor(19, 3);  lcd.write('A');
  if (!(digitalRead(lcdToggle))) {  //  A button to switch the content of the display
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(" Room Temp = " + String(roomTemp) + (char)223 + 'C');
    lcd.setCursor(0, 3);
    lcd.print("Motor Temp = " + String(motorTemp) + (char)223 + 'C');
    lcd.clear();
    while (!(digitalRead(lcdToggle)));
  }
}

void singlePhase() {  //  Device has a single phase supply
  voltageState = 0; //  Voltage not usable
  motorState = 0; //  Turn off motor
  lcd.setCursor(0, 0);
  if (tempState == 0) lcd.print("Sing P & Low TP- Off");  //  Single phase and low temperature
  else if (tempState == 1)  lcd.print("Sing P & Norm TP-Off");  //  Single phase and normal temperature
  else if (tempState == 2)  lcd.print("Sing P & High TP-Off");  //  Single phase and high temperature
}

void doublePhase() {  //  Device has a double phase supply
  voltageState = 0; //  Voltage not usable
  motorState = 0; //  Turn off motor
  lcd.setCursor(0, 0);
  if (tempState == 0) lcd.print("Two P & Low TP - Off");  //  Two phases and low temperature
  else if (tempState == 1) lcd.print("Two P & Norm TP- Off");  //  Two phases and normal temperature
  else if (tempState == 2)  lcd.print("Two P & High TP- Off");  //  Two phases and high temperature
}
