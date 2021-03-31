/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define USMIN 500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2470 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_NUM_MAX 16 //Number of servos attached
#define DEG_TO_RAD 0.01745327
#define DEGREE_TO_PULSE(deg) (USMIN + (float(deg)/180)*(USMAX-USMIN))

const int frequency = 1; // Oscillation frequency of segments.
const int delayms = 7; // Delay between limb movements
const float lag = 0.5712; // Phase lag between segments
const int irpin = 7; // the pin number for IR sensor
const int offset[] = {5, 0, -5, -15, -10, -5, -20, -20, -15, -10, 0, -15, -15, 0, -15, -20};
int amplitude = 30; // Amplitude of the serpentine motion of the snake

void initpos(void) {
    int high = (SERVO_NUM_MAX - 1) / 2;
    for (int i = 0; i < SERVO_NUM_MAX; i++) {
        pwm.writeMicroseconds(i, DEGREE_TO_PULSE(90 + amplitude * cos((high - i) * lag) + offset[i]));      
    }
}

void backward(void) {
    for (int deg = 0; deg < 360; deg++)  {
        delay(delayms);
        float arg = deg * frequency * DEG_TO_RAD;
        int high = (SERVO_NUM_MAX - 1) / 2;
        for (int i = 0; i < SERVO_NUM_MAX; i++) {
            pwm.writeMicroseconds(i, DEGREE_TO_PULSE(90 + amplitude * cos(arg + (high - i) * lag) + offset[i]));
        }
    }
}

void forward(void) {
    for (int deg = 360; deg > 0; deg--)  {
        delay(delayms);
        float arg = deg * frequency * DEG_TO_RAD;
        int high = (SERVO_NUM_MAX - 1) / 2;
        for (int i = 0; i < SERVO_NUM_MAX; i++) {
            pwm.writeMicroseconds(i, DEGREE_TO_PULSE(90 + amplitude * cos(arg + (high - i) * lag) + offset[i]));
        }
    }
}

void turn(void) {
    for (int deg = 0; deg < 180; deg++)  {
        delay(delayms);
        float arg = deg * frequency * DEG_TO_RAD;
        int high = (SERVO_NUM_MAX - 1) / 2;
        for (int i = 0; i < SERVO_NUM_MAX; i++) {
            pwm.writeMicroseconds(i, DEGREE_TO_PULSE(90 + amplitude * cos(arg + (high - i) * lag) + offset[i]));
        }
    }  

    for (int deg = 360; deg > 0; deg--)  {
        delay(delayms);
        float arg = deg * frequency * DEG_TO_RAD;
        int high = (SERVO_NUM_MAX - 1) / 2;
        for (int i = 0; i < SERVO_NUM_MAX; i++) {
            pwm.writeMicroseconds(i, DEGREE_TO_PULSE(90 + amplitude * cos(arg + (high - i) * lag) + offset[i]) + 30);
        }
    }
}

bool blocked(void) {
    return (!digitalRead(irpin));
}

void setup() {
    Serial.begin(9600);
    
    // initialize the IR pin as an input:
    pinMode(irpin, INPUT);
    
    // In theory the internal oscillator is 25MHz but it really isn't
    // that precise. You can 'calibrate' by tweaking this number till
    // you get the frequency you're expecting!
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    // Actuate the snake to its initial position
    initpos();
    delay(1000);
}

void loop() {
    forward();
    if (blocked()) {
        // Move backward and turn before continuing to move forward
        backward();
        turn();
    }
}
