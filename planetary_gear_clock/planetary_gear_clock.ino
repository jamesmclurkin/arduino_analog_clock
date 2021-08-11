/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Stepper.h>

// Pin 13 has an LED connected on most Arduino boards, including this clock
#define PIN_BLINKY_LED    13

#define PIN_HALL_SENSOR   15

#define PIN_BUTTON_SELECT 3

#define PIN_STEPER0_STEP  6
#define PIN_STEPER0_DIR   7

#define STEPER_DELAY     1

// change this to the number of steps on your motor
#define STEPS 2052

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 17, 5, 6, 7);

void setup() {
  // initialize the digital pin as an output.
  pinMode(PIN_BLINKY_LED, OUTPUT);

  pinMode(PIN_HALL_SENSOR, INPUT);
  pinMode(PIN_BUTTON_SELECT, INPUT);


  pinMode(PIN_STEPER0_STEP, OUTPUT);
  pinMode(PIN_STEPER0_DIR, OUTPUT);
  digitalWrite(PIN_STEPER0_STEP, LOW);
  digitalWrite(PIN_STEPER0_DIR, HIGH);

  // set the speed of the motor to 10 RPMs
  stepper.setSpeed(10);
}

#define DIRECTION_COUNT 5000
#define NOON_OFFSET     170

int direction_count = DIRECTION_COUNT;
bool button_select_old = false;
int step_size = -1;
bool done = false;

void loop() {
  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }
  // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }

  if (!done) {
    stepper.step(step_size);
    delay(STEPER_DELAY);
    digitalWrite(PIN_STEPER0_STEP, HIGH);

    //digitalWrite(PIN_BLINKY_LED, LOW);
    stepper.step(step_size);
    delay(STEPER_DELAY);
    digitalWrite(PIN_STEPER0_STEP, LOW);

    bool hall_sensor = !digitalRead(PIN_HALL_SENSOR);
    digitalWrite(PIN_BLINKY_LED, hall_sensor);

    bool button_select = !digitalRead(PIN_BUTTON_SELECT);
    if (button_select && (!button_select_old)) {
      step_size = -1 * step_size;
    }
    button_select_old = button_select;

    if (hall_sensor && (step_size == 1)) {
      //we've found the start of the hall while turning clockwise.  step the offset and stop at noon.
      stepper.step(NOON_OFFSET);
      done = true;
    }
  }

  // direction_count--;
  // if (direction_count == 0) {
  //   direction_count = DIRECTION_COUNT;
  // }
  // if (direction_count > (DIRECTION_COUNT/2)) {
  //   digitalWrite(PIN_STEPER0_DIR, true);
  // } else {
  //   digitalWrite(PIN_STEPER0_DIR, false);
  // }
}
