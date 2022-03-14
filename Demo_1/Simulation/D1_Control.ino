#include <Encoder.h>
#include <Wire.h>


// Motor constants
const int D2 = 4;
const int STATUS_FLAG = 12;
const int MOTOR1_SIGN = 7;
const int MOTOR2_SIGN = 8;
const int MOTOR1_SPEED = 9;
const int MOTOR2_SPEED = 10;

// Timing globals
const int SAMPLE_COUNT = 100;
const int PERIOD = 10; // In milliseconds
unsigned long int time_start = 0; // In milliseconds

// Data globals
long old_position = 0;
double old_angular_position = 0;
double new_angular_position = 0;
double d_angular_position = 0;
double angular_velocity = 0;


long old_position_2 = 0;
double old_angular_position_2 = 0;
double new_angular_position_2 = 0;
double d_angular_position_2 = 0;
double angular_velocity_2 = 0;

double rho = 0;
double phi = 0;
//radius of wheel
double R = .2428;
//distance between wheels
double D = .8989;
bool RUN = true;
double radTurn = 0;
double SPEED = 65;
bool rampDown = false;
bool STOP = false;






// Encoder
Encoder wheel_1(2, 6);
Encoder wheel_2(3, 5);

void drive_straight(double feet) {

  digitalWrite(MOTOR1_SIGN, LOW);
  digitalWrite(MOTOR2_SIGN, HIGH);
  RUN = true;
  STOP = false;
  SPEED = 65;
  while (RUN == true) {
    calc_stats();
    
    analogWrite(MOTOR1_SPEED, SPEED);
    analogWrite(MOTOR2_SPEED, SPEED);
    

    if (rho >= feet && STOP == false) {
      
      RUN = false;
      STOP = true;
      

      analogWrite(MOTOR1_SPEED, 0);
      analogWrite(MOTOR2_SPEED, 0);

    }
    while (phi != 0 && STOP == true ){
if (phi > 0 ) {
   calc_stats();
      digitalWrite(MOTOR1_SIGN, LOW);
      digitalWrite(MOTOR2_SIGN, LOW);
      analogWrite(MOTOR1_SPEED, 35);
      analogWrite(MOTOR2_SPEED, 35);

      
      

    }
    else if (phi < 0 ) {
       calc_stats();
      digitalWrite(MOTOR1_SIGN, HIGH);
      digitalWrite(MOTOR2_SIGN, HIGH);
      analogWrite(MOTOR1_SPEED, 35);
      analogWrite(MOTOR2_SPEED, 35);
     


    }
    

      
    }
    if (phi == 0 && STOP == true) {
      analogWrite(MOTOR1_SPEED, 0);
      analogWrite(MOTOR2_SPEED, 0);
      wheel_1.write(0);
      wheel_2.write(0);
      STOP = false;
    }
  }




  }
  // right turn = clockwise = negative angle
  void rotate_right( double right_turn) {
    RUN = true;
    SPEED = 65;

    digitalWrite(MOTOR1_SIGN, LOW);
    digitalWrite(MOTOR2_SIGN, LOW);

    while (RUN == true) {
      calc_stats();


      analogWrite(MOTOR1_SPEED, SPEED);
      analogWrite(MOTOR2_SPEED, SPEED);


      if (phi < right_turn + 0.2 && rampDown == false) {
        SPEED = 35;
        rampDown = true;
      }
      if (phi <= right_turn ) {
        wheel_1.write(0);
        wheel_2.write(0);
        RUN = false;
        rampDown = false;
        analogWrite(MOTOR1_SPEED, 0);
        analogWrite(MOTOR2_SPEED, 0);

      }

    }

  }
  // left turn = counterclockwise = positive angle
  void rotate_left( double left_turn) {

    RUN = true;
    SPEED = 65;
    digitalWrite(MOTOR1_SIGN, HIGH);
    digitalWrite(MOTOR2_SIGN, HIGH);
    while (RUN == true) {
      calc_stats();
      Serial.print(phi);
      Serial.print(",");
      //Serial.print(new_angular_position_2);

      Serial.print("\n");




      analogWrite(MOTOR1_SPEED, SPEED);
      analogWrite(MOTOR2_SPEED, SPEED);

      if (phi > left_turn - 0.2 && rampDown == false) {
        SPEED = 35;
        rampDown = true;
      }
      if (phi >= left_turn ) {
        wheel_1.write(0);
        wheel_2.write(0);
        RUN = false;
        rampDown = false;
        analogWrite(MOTOR1_SPEED, 0);
        analogWrite(MOTOR2_SPEED, 0);

      }

    }

  }






  void calc_stats() {
    long new_position = wheel_1.read();
    long new_position_2 = wheel_2.read();

    if (new_position == old_position) {
      // Don't need to update any stats as nothing has changed
      return;
    }

    old_position = new_position;
    old_position_2 = new_position_2;

    new_angular_position = (2.00 * 3.1415926 * new_position) / 3200.0;
    new_angular_position_2 = (2.00 * 3.1415926 * new_position_2) / 3200.0;

    d_angular_position = new_angular_position - old_angular_position;
    angular_velocity = (d_angular_position / PERIOD) * 1000.0;

    d_angular_position_2 = new_angular_position_2 - old_angular_position_2;
    angular_velocity_2 = (d_angular_position_2 / PERIOD) * 1000.0;

    rho =   R * ((new_angular_position + new_angular_position_2)/2 );
    phi =   R * ((new_angular_position - new_angular_position_2) / D);

    old_angular_position = new_angular_position;
    old_angular_position_2 = new_angular_position_2;


    /*
      //Serial.print(time_start);
      Serial.print("\t");
      //Serial.print(new_angular_position);
      // Serial.print("\t");
      Serial.print(rho);
      Serial.print(",");
      Serial.print("\n");
    */
  }

  void setup() {
    pinMode(D2, OUTPUT);

    pinMode(MOTOR1_SIGN, OUTPUT);
    pinMode(MOTOR2_SIGN, OUTPUT);

    pinMode(MOTOR1_SPEED, OUTPUT);
    pinMode(MOTOR2_SPEED, OUTPUT);

    pinMode(STATUS_FLAG, INPUT);

    digitalWrite(D2, HIGH);
    digitalWrite(MOTOR1_SIGN, LOW);
    digitalWrite(MOTOR2_SIGN, LOW);

    Serial.begin(9600);
    Wire.begin(0x08);
    Serial.println("Performing encoder test...");


  }

  void loop() {
    

    drive_straight(4);
    delay(5000);
    //rotate_left(1.6);



  }
