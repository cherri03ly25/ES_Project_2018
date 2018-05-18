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
#define LEFTMOST 120
#define RIGHTMOST 60
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
  //for error tracking and debugging
  Serial.begin(9600);
  
  // i2c 
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  //set button pin to input
  START_BUTTON_PORT |= _BV(START_BUTTON_BIT);
      
  // enable steering servo at pin 11
  myservo.attach(11);  //11

  // enable motor servo at pin 6
  motor.attach(6); //6
  DDRK|=(1<<INA)|(1<<INB);  // set motor control pins to output  
}


void loop() {  
  //check if the button pressed for start/stop the motor
  if (start_button_pressed()) {
    if (run == 1) {
      run_stop();
    }
    else {
      run_start();
    }
  } 
}

//i2c command receiver and handler
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
        full_control(msg[1],msg[2],msg[3]);
        break;
        
      default:
        break;        
    }
  
}

//i2c message transmitter 
void sendData() {
  if(cmd == READ){
      byte data[] = {run,s,m};
      Wire.write(data,3);
  }
}

//to stop the motor
void run_stop() {
  if(run == 1){
    PORTK = MSTOP;
    run = 0;
    m = 0;
    motor.write(m);
    s = CENTER;
    myservo.write(s);

  }
}

//to start the motor
void run_start() {
  if(run == 0){
    s = CENTER;
    myservo.write(s);
    m = MIN;
    motor.write(m);
    PORTK = CW;
    run = 1;
  }
  else{
    s = CENTER;
    myservo.write(s);
  }  
}

//to accelerate the motor
void accelerate() {
   if (run) ++m;
   if (m > MAX) m = MAX;
   motor.write(m);
}

//to decelerate the motor 
void decelerate() {
   if (run) --m;
   if (m < MIN) m = MIN;   
   motor.write(m);
}

//to control the servo for right-side steering
void turn_right() {
  s-=2;
  if(s < RIGHTMOST) s = RIGHTMOST;
  myservo.write(s);  
}

//to control the servo for left-side steering
void turn_left() {
  s+=2;
  if(s > LEFTMOST) s = LEFTMOST; 
  myservo.write(s); 
}

//to control the motor for backward run
void run_back(){
  if(run == 0){
    m = MIN;
    motor.write(m);
    PORTK = CCW;
    run = 1;
  }
}
  
// write specific control parameters of steering and speed to the servo and the motor
void full_control(boolean running, byte steering, byte motor_speed) {
  
  //write run and speed values
  if(running) {    
    if (motor_speed < MIN) {
      m = MIN;
    }
    else if (motor_speed > MAX) {
      m = MAX;
    }
    else {
      m = motor_speed;
    }    
    motor.write(m);     
    PORTK = CW;
    run = 1;   
  }
  else{
    PORTK = MSTOP;
    m = 0;
    motor.write(m);     
    run = 0;
  } 
  
  //write steering values
  {
  if(s < RIGHTMOST){ 
    s = RIGHTMOST;
  }
  else if(s > LEFTMOST) {
    s = LEFTMOST;
  }
  else {
    s = steering;
  }
 
  myservo.write(s); 
  }
  
  Serial.println("################");
  Serial.println(run);
  Serial.println(s);
  Serial.println(m);
}

//check if button is pressed for a a short predefined time
int start_button_pressed() {

        if (bit_is_clear(START_BUTTON_PIN, START_BUTTON_BIT)) {

                _delay_ms(BUTTON_PRESS_DELAY_TIME);

                if (bit_is_clear(START_BUTTON_PIN, START_BUTTON_BIT)) return 1;

        }
        
        return 0;
}








