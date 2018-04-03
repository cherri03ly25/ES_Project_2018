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

// i2c communication
#define SLAVE_ADDRESS 0x4a

// bumper
#define BUMPER_PORT PORTA
#define BUMPER_DDR  DDRA
#define BUMPER_PIN  PINA
uint8_t bumper_status = 0b11111111;
uint32_t safety_counter = 0;
#define SAFETY_LIMIT 400000 //270000
 
// steering servo
Servo myservo;  
#define SERVO_PORT  PORTB
#define SERVO_DDR   DDRB
#define STEERING_SERVO PB5 // 11 
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
uint8_t running = 0;


void setup() {
  // i2c
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  //set button pin to input
  START_BUTTON_PORT |= _BV(START_BUTTON_BIT);
  
  // set bumper pins to input
  BUMPER_DDR = 0b00000000;
   
  // steering
  myservo.attach(11);  

  // motor
  motor.attach(6);
  // set motor control pins to output
  DDRK|=(1<<INA)|(1<<INB);  
}

int steering_by_sensors = 1;

void loop() {
  
  //button pressed for start (long press) or stop (short press)
  if (start_button_pressed()) {
    if (running == 1) {
      running = 0;
      PORTK = STOP; // STOP
      m = 0;
      safety_counter = 0;
    }
    else {
      running = 1;
      PORTK = CW;   // CW rotation
      m = MINIMAL;
      safety_counter++;
    }
    motor.write(m);      // set speed
  }
  
  // safe stop
  if (safety_counter > SAFETY_LIMIT) {
    running = 0;
    PORTK = STOP;
    m = 0;
    motor.write(m);
    safety_counter = 0;
  }
  
  if (steering_by_sensors) {
    bumper_status = BUMPER_PIN;
    steering(bumper_status);
  }
  
}

byte msg[3];
byte cmd = 0;

void receiveData(int byteCount) {
  int i=0;
  while(Wire.available()) {
    msg[i] = Wire.read();
    i++;
  }
 
  if (msg[0] == 0) {
    s = msg[1];
    m = msg[2];
    Serial.println(s);
    Serial.println(m);
    steering_by_sensors = 0;
    control(s,m); 
  }
  else {
    cmd = msg[0]; 
  }
}



void sendData() {
  byte data[] = {s,m};
  switch (cmd) {
    case 1:
      Wire.write(data,2);
      break;     
  }
}

//control steering and motor speed according to RPi's command
void control(byte s, byte m){
  myservo.write(s);

  if (m == 0) {
    PORTK = STOP; // STOP
    motor.write(0);
    safety_counter = 0;
    running = 0;
    steering_by_sensors = 1;
  }
  else if (m == 1) {
    PORTK = CCW;   // CW rotation
    motor.write(FAST);
    safety_counter ++;
    running = 1;
  }
  else {
    PORTK = CW;
    motor.write(m);
    safety_counter ++;
    running = 1;
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

void accelerate() {
   if (running) {
      m = m+1;
      PORTK = CW;   // CW rotation
   }
}

//test

