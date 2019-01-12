#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino  pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     10  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

char receivedChar;
int Led = 0;
boolean newData = false;
String str1, resultStr;
float aReading;
float WLReading;
int pin_Pressure=2;
int pin_WD=1;
float minWD=250;
unsigned int WL_uS = 0;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
// put your setup code here, to run once:
Serial.begin(9600);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(3,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  str1="the data is ";

  
  for (int i =3; i<7; i++) {
    
  
    digitalWrite(i, HIGH);
    delay(500);
    digitalWrite(i, LOW);
  }
}

void loop() {
// put your main code here, to run repeatedly:

  recvInfo();
  ProcessRequest();


//  delay(500);
}

void recvInfo() {

  if (Serial.available() > 0) {

    receivedChar = Serial.read();
    newData = true;
    
  }
  
}

void ProcessRequest() {

  int req = receivedChar - '0';
  int i;  
  while(newData == true) {

    if (req == 1) { //water detection
      aReading = analogRead(pin_WD);
      if (aReading > minWD) {
        resultStr = "WD,YES,";
      }
      else {
        resultStr = "WD,NO,";
      }
      
      resultStr = resultStr + aReading;
      Serial.println(resultStr);
    }
    else if (req == 2) {  //water level requested. using HC-SR04 ultrasonic sensor
      WL_uS = sonar.ping();
      if (WL_uS == 0) {
        resultStr = "WL,NO,";
      }
      else {
        resultStr = "WL,YES,";
      }
      WLReading = WL_uS/US_ROUNDTRIP_CM;
      resultStr = resultStr + WLReading;
      Serial.println(resultStr);
    }
    else if (req == 3) {
//          for (i=1;i<50;i++) {
            aReading = analogRead(pin_Pressure);             //read from the ADC
            resultStr="WP,YES,";
            resultStr= resultStr + aReading;
            Serial.println(resultStr);//data that is being Sent
//            delay(1000);
//          }
    }
    Led = 3;
    digitalWrite(Led, HIGH);
    delay(200);
    digitalWrite(Led, LOW);

    newData = false;
    
  }
  
}
