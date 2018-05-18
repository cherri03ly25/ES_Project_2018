//https://forum.arduino.cc/index.php?topic=539927.0
//https://electronics.stackexchange.com/questions/89/how-do-i-measure-the-rpm-of-a-wheel?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
//http://engineerexperiences.com/tachometer-using-arduino.htm

#define COUNT_PERIOD 500
#define PI 3.1416
#define WHEEL_RADIUS 0.03; //in meters
unsigned long counts;
unsigned long start_time;
unsigned int rps;
float car_speed;

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

void setup()
{
  Serial.begin(115200);
  TCCR5A = 0; //initialize Timer5
  TCCR5B = 0;
  pinMode( 47, INPUT_PULLUP); //external source pin for timer5
  timer5_reset();
}

void loop()
{
  if (millis() - start_time >= COUNT_PERIOD)
  {
    rps = get_rps();
    car_speed = get_speed();
    timer5_reset();
  }
}
