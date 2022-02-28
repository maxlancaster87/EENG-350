//This program takes an input from a Raspberry Pi board which is read by this program which will be implement it 
// on the arduino board. The data that reads from the Pi board is a single byte, in other words just one integer is 
// read by this program once it has been received by the Pi board, and is given the name quadrantNum. Once this data 
// has been received then we use a motor to do specific tasks such as rotating in specific angles depending on the 
// number/data received by the Pi board, in other words depending on the integer that has been assigned by quadrantNum. 
// In summary, this program acts as the motor controller for a motor to do specific tasks depending on given data 
// received by a raspberry pi board.

#include <Wire.h> //Library used to receive and read data that is been inputted to this address/identifier/board
#include <Encoder.h> //Library used for the encoder part of the motor 

#define SLAVE_ADDRESS 0x04 //Adress of this arduino board acting as a slave for the pi board

int quadrantNum; //Quadrany Number is defined as global variable

Encoder knob(2, 6);
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached


void setup() {

  pinMode(12, INPUT); //Input pin
  pinMode(4, OUTPUT); //Motor Off/On(ENABLE) pin
  pinMode(7, OUTPUT); //Motor Direction Pin(one motor will be use which on this pin)
  pinMode(8, OUTPUT); // Motor Direction pin(this motor will not be used)
  pinMode(9 , OUTPUT); //Motor Speed pin(only this one will be used)
  pinMode(10, OUTPUT); //Motor Speed pin(this one will not be used)

  Serial.begin(9600); // start serial for output on serial monitor of the arduino board

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
}

long positionKnob  = -999; //makes sure the initial "previous position" will not match new position
long newVal = 0; // This variable is used for the current encoder position

void loop() {
  
  digitalWrite(4, HIGH); // Motor OFF/On pin & state
  

  float Rad; // radians
  newVal = knob.read(); //Reading the new vals which is the counts by the encoder library



  if (newVal != positionKnob) { // If counts are not in the position desired 
    Serial.print("Count  = ");
    Serial.print(newVal);
    Serial.println();

    positionKnob = newVal;

    Rad = newVal * (2 * 3.1415926535897932384626433832795) / 3200; //Conversion from counts to radians

  }

  //--------------------- The following if statatements sets the motor position based on the quadrant ----------------------------------------
  // -------------------- number received from the pi board to the arduino. It uses the encoder library --------------------------------------
  // -------------------- and counts to set the correct position desired.                             ----------------------------------------
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

  while (Wire.available()) { // While there's available data read it to the arduino using the wire library
    quadrantNum = Wire.read(); //Read the data
    Serial.print("data received: "); //Print the received data to the serial monitor
    Serial.println(quadrantNum);
  }

}
