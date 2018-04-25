#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>  
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>

//tachometer
#define TACHOMETER PL2 //47

// i2c communication
#define SLAVE_ADDRESS 0x4a

// bumper
#define BUMPER_PORT PORTA
#define BUMPER_DDR  DDRA
#define BUMPER_PIN  PINA
uint8_t bumper_status = 0b11111111;

//safe stop
uint32_t safety_counter = 0;
#define SAFETY_LIMIT 500000
 
// steering servo
Servo myservo;  
#define SERVO_PORT  PORTB
#define SERVO_DDR   DDRB
#define STEERING_SERVO PB5 //11
//define steering speed for s
#define LEFT4 125
#define LEFT3 115
#define LEFT2 100
#define LEFT1 95
#define NORMAL 90
#define RIGHT1 85
#define RIGHT2 80
#define RIGHT3 65
#define RIGHT4 55
byte s = NORMAL; // steering val

// motor
Servo motor;
#define MOTOR_PWM PH3  // 6
#define INA PK0
#define INB PK1
#define CW 0b00000001
#define STOP 0b00000000
#define CCW 0b00000010
//define motor speed for m (m = 0: stop running)
#define MINIMAL 70
#define SLOW 80
#define MEDIUM 85
#define FAST 90
byte m = 0; // motor val

//button to start/stop running
#define START_BUTTON_PORT PORTE
#define START_BUTTON_PIN PINE
#define START_BUTTON_BIT PE5
#define BUTTON_PRESS_DELAY_TIME 50

//global running status
uint8_t running = 0;

//tachometer
#define COUNT_PERIOD 2000
#define PI 3.1416
#define WHEEL_RADIUS 0.03; //in meters
unsigned long counts;
unsigned long start_time;
unsigned int rps;
float car_speed;


void setup() {
  Serial.begin(9600);
  /*
  //tachometer
  TCCR5A = 0; //initialize Timer5
  TCCR5B = 0;
  pinMode( 47, INPUT_PULLUP); //external source pin for timer5
  timer5_reset();*/
  
  // i2c
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  //set button pin to input
  START_BUTTON_PORT |= _BV(START_BUTTON_BIT);
  
  // set bumper pins to input
  BUMPER_DDR = 0b00000000;
        
  // steering
  myservo.attach(11);  //11

  // motor
  motor.attach(6); //6
  // set motor control pins to output
  DDRK|=(1<<INA)|(1<<INB);  
}

int steering_by_sensors = 0;

void loop() {
  //button pressed for start (long press) or stop (short press)
  if (start_button_pressed()) {
    if (running == 1) {
      running = 0;
    }
    else {
      running = 1;
      m = MINIMAL;
      motor.write(m); 
    }
     // set speed
  }
  
  //check running status always
  if(running){
    PORTK = CW;
    safety_counter++;
  }
  else{
    PORTK = STOP;
    m = 0;
    motor.write(m); 
    safety_counter = 0;
  }
  
  // safe stop
//  if(safety_counter > SAFETY_LIMIT) {
//    running = 0;
//  }
  
  //check if sensors on
  if (steering_by_sensors) {
    bumper_status = BUMPER_PIN;
    steering(bumper_status);
  }
  /*
  if (millis() - start_time >= COUNT_PERIOD)
  {
    rps = get_rps();
    car_speed = get_speed();
    timer5_reset();
  }*/
  
}

byte cmd = 0;

void receiveData(int byteCount) {
  byte msg[3];
  int i=0;
  while(Wire.available()) {
    msg[i] = Wire.read();
    i++;
  }
  
  cmd=msg[0];
  
  if (cmd <= 1) {
    s = msg[1];
    m = msg[2];
    Serial.println(s);
    Serial.println(m);
    //steering_by_sensors = 0;
    control(s,m);  
  }
}



void sendData() {
  switch (cmd) {
    case 2:
      byte data[] = {running,s,m};
      Wire.write(data,3);
      break;
//    case 3:
//      byte data2[] = {rps, car_speed};
//      Wire.write(data,2);
//      break;   
  }
}

//control steering and motor speed according to RPi's command
void control(byte s, byte m){
  if(s>=60 && s<=120){
    myservo.write(s);
    switch (cmd){
      case 0:
        running = 0;
        //steering_by_sensors = 1;
        break;
      case 1:
        running = 1;
        if (m <= 0 && m <= 100){
          motor.write(m);
        }
        break;
    }
  }
}

//buton pressed (return 1 for long press, 0 for short press)
int start_button_pressed() {

        if (bit_is_clear(START_BUTTON_PIN, START_BUTTON_BIT)) {

                _delay_ms(BUTTON_PRESS_DELAY_TIME);

                if (bit_is_clear(START_BUTTON_PIN, START_BUTTON_BIT)) return 1;

        }

        return 0;

}

void timer5_reset(){
  TCNT5 = 0;
  //set external clock source pin D5 rising edge
  TCCR5B =  bit (CS50) | bit (CS51) | bit (CS52);
  start_time = millis();
}

unsigned int get_rps(){
    TCCR5B = 0; //stop counter
    counts = TCNT5; //frequency limited by unsigned int TCNT5 without rollover counts
    return counts*1000/COUNT_PERIOD;
}

float get_speed(){ //in meters/s
  float s = rps * PI * WHEEL_RADIUS;
  return s;
}


void accelerate() {
   if (running) {
      m = m+1;
      PORTK = CW;   // CW rotation
   }
   
   if (m > FAST) {
      m = FAST;
   }
}
 
// use bumber status to adjust steering and motor speed
void steering(uint8_t bumper_status) {
      if ((bumper_status ^ 0b01111111)==0) {
        s = LEFT4;
        m = MINIMAL;   
        safety_counter = 0;
      }
      else if ((bumper_status ^ 0b10111111)==0) {
        s = LEFT3;
        m = SLOW;
        safety_counter = 0;
      }
      else if ((bumper_status ^ 0b11011111)==0) {
        s = LEFT2;
        m = MEDIUM;
        safety_counter = 0;
      }
      else if ((bumper_status ^ 0b11101111)==0) {
        s = LEFT1;
        m = FAST;
        safety_counter = 0;
      }
      else if ((bumper_status ^ 0b11110111)==0) {
        s = RIGHT1;
        m = FAST;
        safety_counter = 0;
      }
      else if ((bumper_status ^ 0b11111011)==0) {
        s = RIGHT2;
        m = MEDIUM;
        safety_counter = 0;
      }
      else if ((bumper_status ^ 0b11111101)==0) {
        s = RIGHT3;
        m = SLOW;
        safety_counter = 0;
      }
      else if ((bumper_status ^ 0b11111110)==0) {
        s = RIGHT4;
        m = MINIMAL;
        safety_counter = 0;
      }      
      else {
         safety_counter++;
      }
    
// set limits for m in case acceleration() is used

      if (m > FAST) {
         m = FAST;
      }
      
      myservo.write(s); //set steering angle
      motor.write(m); //set motor speed
      
}





