#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>  
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif


// i2c communication
#define SLAVE_ADDRESS 0x4a
 
// steering servo
Servo myservo;  
#define SERVO_PORT  PORTB
#define SERVO_DDR   DDRB
#define STEERING_SERVO PB5 //11
//steering angles
#define LEFTMOST 125
#define RIGHTMOST 55
#define CENTER 90
byte s = CENTER;

// motor
Servo motor;
#define MOTOR_PWM PH3  // 6
#define INA PK0
#define INB PK1
#define CW 0b00000001
#define MSTOP 0b00000000
#define CCW 0b00000010
//motor speed
#define MIN 40
#define MAX 100
byte m = 0; // motor val

//button to start/stop running
#define START_BUTTON_PORT PORTE
#define START_BUTTON_PIN PINE
#define START_BUTTON_BIT PE5
#define BUTTON_PRESS_DELAY_TIME 50

//global running status
boolean run = 0;

//commands
#define STOP 0
#define START 1
#define FASTER 2
#define SLOWER 3
#define RIGHT 4
#define LEFT 5
#define BACK 6
#define WRITE 7
#define READ 8

byte cmd = -1;

void setup() {
  // i2c 
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  //set button pin to input
  START_BUTTON_PORT |= _BV(START_BUTTON_BIT);
      
  // steering
  myservo.attach(11);  //11

  // motor
  motor.attach(6); //6
  DDRK|=(1<<INA)|(1<<INB);  // set motor control pins to output  
}


void loop() {  
  //button pressed for start (long press) or stop (short press)
  if (start_button_pressed()) {
    if (run == 1) {
//      run = 0;
//      s = CENTER;
//      m = 0;
//      myservo.write(s);
//      motor.write(m);
//      PORTK = MSTOP;
      run_stop();
    }
    else {
//      run = 1;
//      s = CENTER;
//      m = MIN;
//      myservo.write(s);
//      motor.write(m);
//      PORTK = CW;
      run_start();
    }
  }
}


void receiveData(int byteCount) {
  byte msg[4]={0};
  int i=0;
  while(Wire.available()) {
    msg[i] = Wire.read();
    i++;
  }
  
  cmd=msg[0];
  
    switch (cmd) {
      
      case STOP:
        run_stop();
        break;
        
      case START:
        run_start();
        break;
        
      case FASTER:
        accelerate();
        break;
        
      case SLOWER:
        decelerate();
        break;

      case RIGHT:
        turn_right();
        break;
        
      case LEFT:
        turn_left();
        break;
        
      case BACK: 
        run_back();
        break;
      
      case WRITE:
        run = msg[1];
        s = msg[2];
        m = msg[3];
        write_data(run,s,m);
        break;
        
      default:
        break;        
    } 
  
}

void sendData() {
  if(cmd == READ){
      byte data[] = {run,s,m};
      Wire.write(data,3);
  }
}

void run_stop() {
  if(run==1){
    run = 0;
    s = CENTER;
    m = 0;
    myservo.write(s);
    motor.write(m);
    PORTK = MSTOP;
  }
}

void run_start() {
  if(run==0){
    run = 1;
    s = CENTER;
    m = MIN;
    myservo.write(s);
    motor.write(m);
    PORTK = CW;
  }
  else{
    s = CENTER;
    myservo.write(s);
  }  
}

void accelerate() {
   if (run) ++m;
   if (m > MAX) m = MAX;
   motor.write(m);
}
 
void decelerate() {
   if (run) --m;
   if (m < MIN) m = MIN;   
   motor.write(m);
}

void turn_right() {
  s -= 5;
  if(s < RIGHTMOST) s = RIGHTMOST;
  myservo.write(s);  
}

void turn_left() {
  s += 5;
  if(s > LEFTMOST) s = LEFTMOST; 
  myservo.write(s); 
}

void run_back(){
  if(run == 0){
    run = 1;
    m = MIN;
    motor.write(m);
    PORTK = CCW;
  }
}
  
  

void write_data(boolean run, byte s, byte m) {
  if (m < MIN) m = MIN;
  if (m > MAX) m = MAX;
  if(s < RIGHTMOST) s = RIGHTMOST;
  if(s > LEFTMOST) s = LEFTMOST;
 
  if(run) {
    PORTK = CW;
  }
  else{
    PORTK = MSTOP;
    m = 0;
  }
  
  motor.write(m); 
  myservo.write(s); 
  
}

//buton pressed (return 1 for long press, 0 for short press)
int start_button_pressed() {

        if (bit_is_clear(START_BUTTON_PIN, START_BUTTON_BIT)) {

                _delay_ms(BUTTON_PRESS_DELAY_TIME);

                if (bit_is_clear(START_BUTTON_PIN, START_BUTTON_BIT)) return 1;

        }
        
        return 0;
}







