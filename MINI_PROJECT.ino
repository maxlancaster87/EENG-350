#include <Wire.h>
#include <Encoder.h>

#define SLAVE_ADDRESS 0x04

int quadrantNum;
Encoder knob(2, 6);
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached


void setup() {

  pinMode(12, INPUT);
  pinMode(4, OUTPUT); //Motor Off/On(ENABLE) pin
  pinMode(7, OUTPUT); //Motor Direction Pin
  pinMode(8, OUTPUT); // '' Dir pin
  pinMode(9 , OUTPUT); //MotorSpeed pin
  pinMode(10, OUTPUT); // '' Speed pin

  Serial.begin(9600); // start serial for output

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
}

long positionKnob  = -999;
long newVal = 0;

void loop() {
  //analogWrite(9, 125); //Motor speed pin & speed
  digitalWrite(4, HIGH); // Motor OFF/On pin & state
  

  float Rad;
  newVal = knob.read();



  if (newVal != positionKnob) {
    Serial.print("Count  = ");
    Serial.print(newVal);
    Serial.println();

    positionKnob = newVal;



    Rad = newVal * (2 * 3.1415926535897932384626433832795) / 3200;

    //Serial.println("Rad =  ");
    //Serial.print(Rad);
    //Serial.println();


  }

  if (quadrantNum == 1) {
    if (newVal > 3200) {
      digitalWrite(7, LOW);
      analogWrite(9, 30);
    } else {
      digitalWrite(7, HIGH);
      analogWrite(9, 30);
    }

    if (newVal >= 3150 && newVal <= 3250) {
      analogWrite(9, 0);
      
    }
  }


  if (quadrantNum == 2) {
    if (newVal > 800) {
      digitalWrite(7, LOW);
      analogWrite(9, 30);
    } else {
      digitalWrite(7, HIGH);
      analogWrite(9, 30);
    }
    if (newVal >= 750 && newVal <= 850) {
      analogWrite(9, 0);
      
    }
  }



  if (quadrantNum == 3) {
    if (newVal > 1600) {
      digitalWrite(7, LOW);
      analogWrite(9, 30);
    } else {
      digitalWrite(7, HIGH);
      analogWrite(9, 30);
    }
    if (newVal >= 1550 && newVal <= 1650) {
      analogWrite(9, 0);
      
    }
  }


  if (quadrantNum == 4) {
    if (newVal > 2400) {
      digitalWrite(7, LOW);
      analogWrite(9, 30);
    } else {
      digitalWrite(7, HIGH);
      analogWrite(9, 30);
    }
    if (newVal >= 2350 && newVal <= 2450) {
      analogWrite(9, 0);
      

    }
  }
}

// callback for received data
void receiveData(int byteCount) {

  while (Wire.available()) {
    quadrantNum = Wire.read();
    Serial.print("data received: ");
    Serial.println(quadrantNum);




  }







}
