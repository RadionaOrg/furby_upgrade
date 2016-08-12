#include <TimedAction.h>
#include <TimerOne.h>
#include <NewPing.h>

#define pwmRegister OCR1A    // the logical pin, can be set to OCR1B

#define B_led1       A2 // RGB led1 B leg...
#define G_led1       A3 // RGB led1 G leg...
#define R_led1        6 // RGB led1 R leg...
#define B_led2        3 // RGB led2 B leg...
#define G_led2        4 // RGB led2 G leg...
#define R_led2        5 // RGB led2 R leg...
#define furbyLedIN    7 // from furby leds to pin 7
#define furbyClock    9 // from arduino pin 9 to furby clock
#define TRIGGER_PIN  A0 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A1 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define furbyTail    12 // from arduino pin 12 to furby tail button

#define MAX_DISTANCE 900 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

TimedAction timedAction = TimedAction(2000,furby_tail); // If someone is close to furby (90cm) every 2 sec we will pull furby tail

//used to set clock period
long period = 30; // the period in microseconds
int duty = 512; // duty as a range from 0 to 1024, 512 is 50% duty cycle
int prescale[] = {0, 1, 8, 64, 256, 1024}; // the range of prescale values

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
    Serial.begin(115200);        // start the serial port
    pinMode(furbyClock, OUTPUT); 
    pinMode(furbyTail, OUTPUT); 
    
    pinMode(B_led1, OUTPUT); 
    pinMode(G_led1, OUTPUT); 
    pinMode(R_led1, OUTPUT); 
    pinMode(B_led2, OUTPUT); 
    pinMode(G_led2, OUTPUT); 
    pinMode(R_led2, OUTPUT); 
    
    pinMode(furbyLedIN, INPUT); 
        
    Timer1.initialize(period); // initialize timer1, 30 microseconds
    Timer1.pwm(furbyClock, duty); // setup pwm on pin 9, 50% duty cycle
}

void loop() {
  delay(300);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping: ");         // Serial used for debugging
  long pinging_median = sonar.ping_median(5);
  int ping_median_cm = sonar.convert_cm(pinging_median);
  Serial.print(ping_median_cm); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
if (ping_median_cm>6){
  period = map(ping_median_cm, 1, 199, 15, 90); // Map distance from 1- 199 to get period from 15-90
  Serial.print("Period: ");
  Serial.println(period);
  Timer1.setPeriod(period);
  Timer1.setPwmDuty(furbyClock, duty); // don't change the duty cycle 
  if (ping_median_cm<90) // if someone gets close to furby he will go crazy ...
  {
  timedAction.check(); //chack if 2 sec. are passed
  }
  }
}

// setting furby clock
bool setPulseWidth(long microseconds) {
    bool ret = false;
    int prescaleValue = prescale[Timer1.clockSelectBits];
    // calculate time per tick in ns
    long precision = (F_CPU / 128000) * prescaleValue;
    period = precision * ICR1 / 1000; // period in microseconds
    if (microseconds < period) {
        duty = map(microseconds, 0, period, 0, 1024);
        if (duty < 1)
            duty = 1;
        if (microseconds > 0 && duty < RESOLUTION) {
            Timer1.pwm(furbyClock, duty);
            ret = true;
        }
    }
    return ret;
}

// Do the tailing
void furby_tail(){
  digitalWrite(furbyTail,LOW);
  delay(100);
  digitalWrite(furbyTail,HIGH);
  Serial.println("Tail");
}
