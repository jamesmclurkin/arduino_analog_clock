#include <LedControl.h>
#include <Wire.h>
#include <RtcDS1307.h>
#include <Servo.h>
#include <EEPROM.h>

// example RTC code modified from :
// https://github.com/Makuna/Rtc/blob/master/examples/DS1307_Simple/DS1307_Simple.ino

// LED Display Code from:
// https://playground.arduino.cc/Main/LedControl/

// Daylight Savings time calculation:
// https://stackoverflow.com/questions/5590429/calculating-daylight-saving-time-from-only-date

// Pin 13 has an LED connected on most Arduino boards, including this clock
#define PIN_BLINKY_LED    13

// System constants
#define LOOP_PERIOD       10
#define COLON_BLINK_TIME (100 / LOOP_PERIOD)
#define BUTTON_HOLD_TIME (1000 / LOOP_PERIOD)

// LED Controller pins
#define PIN_LED_CONTROLLER_DATA   10
#define PIN_LED_CONTROLLER_CLK    9
#define PIN_LED_CONTROLLER_LOAD   8

#define LED_BRIGHTNESS_MIN        0
#define LED_BRIGHTNESS_MAX        16
#define LED_BRIGHTNESS_DEFAULT    6

// Servo pins and constants
#define PIN_SERVO_HOUR            6
#define PIN_SERVO_MINUTE          5

#define SERVO_MIN         1
#define SERVO_MAX         180
#define SERVO_MIN_DEFAULT 20
#define SERVO_MAX_DEFAULT 170

// Button pin setup. We create button UI state variables below
#define PIN_BUTTON_UP 4
#define PIN_BUTTON_DOWN 2
#define PIN_BUTTON_SELECT 3

// Light sensor photo resistor
#define PIN_LIGHT_SENSOR            A0

#define LIGHT_SENSOR_AMBIENT        950
#define LIGHT_SENSOR_GAIN           100

#define LIGHT_SENSOR_LPF            100.0


// Configuration information related to an individual servo. We 
// store a min and max value.
struct ServoConfig {
  uint8_t min;
  uint8_t max;
};

// An aggregate of all configuration we need to store. We store servo
// configuration for each servo, and a display brightness.
struct Configuration {
  ServoConfig hour_servo;
  ServoConfig minute_servo;
  uint8_t display_brightness;
  bool DST_when_time_set;

  // Returns a sensible set of default configuration, for use only on fresh
  // boards where we've never performed a calibration, or if the calibration is
  // lost due to memory corruption.
  static Configuration Default() {
    Configuration c;
    c.hour_servo.min = SERVO_MIN_DEFAULT;
    c.hour_servo.max = SERVO_MAX_DEFAULT;
    c.minute_servo.min = SERVO_MIN_DEFAULT;
    c.minute_servo.max = SERVO_MAX_DEFAULT;
    c.display_brightness = LED_BRIGHTNESS_DEFAULT;
    c.DST_when_time_set = false;
    return c;
  }
};

// We serialize the configuration to nonvolatile memory as a flat binary blob
// with a checksum.
struct SavedConfiguration {
  Configuration config;
  uint16_t checksum;
};

// Fletcher16 is a simple checksum algorithm which is sensitive to
// byte ordering.
uint16_t Fletcher16(const void *p, size_t size) {
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;
  const uint8_t *data = static_cast<const uint8_t *>(p);

  for (int i = 0; i < size; ++i) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return (sum1 << 8) | (sum2 & 0xff);
}

// Writes the given configuration object into EEPROM, along with a checksum so
// that we can ensure it's valid when it is read out again.
void WriteConfigurationToEeprom(const Configuration &config) {
  SavedConfiguration serialized;
  serialized.config = config;
  serialized.checksum = Fletcher16(&config, sizeof(Configuration));

  const uint8_t *data = reinterpret_cast<const uint8_t *>(&serialized);
  for (int i = 0; i < sizeof(serialized); ++i) {
    EEPROM.write(i, data[i]);
  }
}

// Read a serialized configuration object out of EEPROM, and verify the checksum
// is correct. If the checksum is not correct, this may be due to EEPROM
// corruption or a fresh IC which has never had any configuration stored.
bool LoadConfigurationFromEeprom(Configuration *config) {
  SavedConfiguration serialized;
  uint8_t *data = reinterpret_cast<uint8_t *>(&serialized);
  for (int i = 0; i < sizeof(serialized); ++i) {
    data[i] = EEPROM.read(i);
  }

  const uint16_t checksum = Fletcher16(&serialized.config, sizeof(Configuration));
  if (checksum != serialized.checksum) {
    return false;
  }
  *config = serialized.config;
  return true;
}

// Factory calibration stored in non-volatile memory. We keep an in-memory
// working copy here, and serialize it to EEPROM when there is a change.
Configuration config;



// The button object holds state related to an individual button, and provides
// common logic for reading, debouncing, and edge detection on buttons.
struct Button {
  int pin;
  bool pressed;
  bool pressed_prev;
  uint16_t held_count;

  // Initializes the button object to read from a button connected to the given
  // pin.
  void Init(int button_pin) {
    pinMode(button_pin, INPUT);
    pin = button_pin;
    pressed = false;
    pressed_prev = false;
    held_count = 0;
  }

  // Read the button pin and update the internal state, looking for edges and
  // other button events.
  void Update() {
    // Detect the edge and store it for press detection.
    pressed_prev = pressed;
    pressed = !digitalRead(pin);
    if (pressed) {
      held_count++;
      held_count = constrain(held_count, 0, BUTTON_HOLD_TIME);
    } else {
      held_count = 0;
    }
  }

  // Returns true if the button was just pressed.
  bool just_pressed() const { return (pressed && !pressed_prev); }

  // Returns true if the button was just released.
  bool just_released() const { return (!pressed && pressed_prev); }

  // Returns true if the button has been held down long enough for it to be
  // considered intentional.
  bool held() const { return (held_count == BUTTON_HOLD_TIME); }
};

// Button objects to scan and debounce the 3 buttons.
Button select_button;
Button up_button;
Button down_button;


// Light sensor
float light_sensor_lpf = LIGHT_SENSOR_AMBIENT;

void light_sensor_update() {
  int light_sensor_val = analogRead(PIN_LIGHT_SENSOR);
  light_sensor_lpf = ((float)light_sensor_val + (light_sensor_lpf * (LIGHT_SENSOR_LPF - 1.0))) / LIGHT_SENSOR_LPF;
}

int brightness_adjust() {
  // compute the light sensor brightness adjustment
  float brightness_adjust;
  brightness_adjust = (light_sensor_lpf - (float)(LIGHT_SENSOR_AMBIENT)) / (float)(LIGHT_SENSOR_GAIN) ;
  return (int)brightness_adjust;
}


// Create a new LedControl object.
// We use pins 8, 9 and 10 on the Arduino for the SPI interface
// Pin 10 is connected to the DATA IN-pin of the first MAX72XX
// Pin 9 is connected to the CLK-pin of the first MAX72XX
// Pin 8 is connected to the LOAD(/CS)-pin of the first MAX72XX
// There will only be a single MAX72XX attached to the arduino
LedControl display = LedControl(PIN_LED_CONTROLLER_DATA, PIN_LED_CONTROLLER_CLK,
                                PIN_LED_CONTROLLER_LOAD, 1);

// RTC setup.  We use a DS1307 RTC
RtcDS1307<TwoWire> Rtc(Wire);

// Maps the day IDs 0-6 to human readable day strings starting with Sunday == 0.
const char *DayOfWeekString(int DoW) {
  switch (DoW) {
  case 0:
    return "Sun";
  case 1:
    return "Mon";
  case 2:
    return "Tue";
  case 3:
    return "Wed";
  case 4:
    return "Thu";
  case 5:
    return "Fri";
  case 6:
    return "Sat";
  default:
    return "---";
  }
}


bool IsDST(const RtcDateTime *time) {
  int day = time->Day();
  int month = time->Month();
  int dow = time->DayOfWeek();

  //January, february, and december are out.
  if (month < 3 || month > 11) { return false; }
  //April to October are in
  if (month > 3 && month < 11) { return true; }
  int previousSunday = day - dow;
  //In march, we are DST if our previous sunday was on or after the 8th.
  if (month == 3) { return previousSunday >= 8; }
  //In november we must be before the first sunday to be dst.
  //That means the previous sunday must be before the 1st.
  return previousSunday <= 0;
}


uint8_t GetAdjustedHour(const RtcDateTime *timePtr, bool *pmPtr) {
  int8_t hour = (int8_t)timePtr->Hour();

  // handle DST silliness
  bool DSTNow = IsDST(timePtr);
  //Serial.print("(DST");
  if (DSTNow != config.DST_when_time_set) {
    //Serial.print("a");
    if (DSTNow) {
      // we are in DST, but the time on the RTC is standard time.  Advance one hour
      hour++;
      //Serial.print("b");
    } else {
      // we are in standard time, but the time on the RTC is DST.  Rewind one hour
      hour--;
      //Serial.print("c");
    }
  }
  //Serial.print(")");

  //wrap the hour around 24
  if (hour < 0) {
    hour += 24;
  }
  if (hour >23) {
    hour -= 24;
  }

  // handle 12-hour time silliness
  *pmPtr = false;
  if (hour == 0) {
    hour = 12;
  } else if (hour > 12) {
    hour = hour - 12;
    *pmPtr = true;
  }
  return (uint8_t)hour;
}



// Formats and writes the given time to the LED display.
void DisplayTime(const RtcDateTime *timePtr) {
  bool pm;
  uint8_t hour = GetAdjustedHour(timePtr, &pm);
  uint8_t minute = timePtr->Minute();

  // Hide the hour 10's place if the hour is less than 10
  if (hour >= 10) {
    display.setDigit(0, 0, 1, false);
  } else {
    display.setChar(0, 0, ' ', false);
  }
  display.setDigit(0, 1, hour % 10, false);
  display.setDigit(0, 2, minute / 10, false);
  display.setDigit(0, 3, minute % 10, false);
}


// Formats and prints the given time to the UART.
void PrintTime(const RtcDateTime *timePtr) {
  bool pm;
  uint8_t hour = GetAdjustedHour(timePtr, &pm);
  uint8_t minute = timePtr->Minute();
  uint8_t second = timePtr->Second();

  // format a descriptive time/date output with DOW and 12-hour format
  Serial.print(DayOfWeekString(timePtr->DayOfWeek()));
  Serial.print(" ");
  Serial.print(timePtr->Year(), DEC);
  Serial.print("-");
  Serial.print(timePtr->Month(), DEC);
  Serial.print("-");
  Serial.print(timePtr->Day(), DEC);
  Serial.print(" ");
  Serial.print(hour, DEC);
  Serial.print(":");
  if (minute < 10) {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second < 10) {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  if (pm) {
    Serial.print(" pm");
  } else {
    Serial.print(" am");
  }

  if (IsDST(timePtr)) {
    Serial.print(" dst");
  }

  Serial.print("  light:");
  //Serial.print(analogRead(PIN_LIGHT_SENSOR), DEC);
  //Serial.print(",");
  Serial.print(light_sensor_lpf, 1);
  Serial.print(",");
  Serial.print(brightness_adjust());

  if (up_button.just_pressed()) {
    Serial.print("  up");
  } else {
    Serial.print("    ");
  }
  if (select_button.just_pressed()) {
    Serial.print(" select");
  } else {
    Serial.print("       ");
  }
  if (down_button.just_pressed()) {
    Serial.print(" down");
  } else {
    Serial.print("     ");
  }

  Serial.println("");
}




// Two servo objects to control the servo positions.
Servo servo_hour;
Servo servo_minute;

// Writes the given stage ID and 8 bit value to the LED display, used for
// showing which step of the calibration process we are on.
void DisplayCalibrationStage(char stage, uint8_t value) {
  display.setChar(0, 0, stage, false);
  for (int i = 0; i < 3; ++i) {
    display.setDigit(0, 3 - i, value % 10, false);
    value /= 10;
  }
}

void DoServoCalibration(char stage, Servo *servo, uint8_t *val) {
  while (true) {
    // Update the buttons, detecting any new user input.
    up_button.Update();
    down_button.Update();
    select_button.Update();

    if (up_button.just_pressed() || up_button.held()) {
      (*val)++;
    }
    if (down_button.just_pressed() || down_button.held()) {
      (*val)--;
    }
    if (select_button.just_pressed()) {
      break;
    }
    *val = constrain(*val, SERVO_MIN, SERVO_MAX);
    DisplayCalibrationStage(stage, *val);
    servo->write(*val);

    delay(LOOP_PERIOD);
  }
}

// Run a calibration routine to find the servo limits, commiting our changes to
// EEPROM when we're done.
void DoCalibration() {
  Serial.println("Calibrating hour 0 pos: Position a");
  DoServoCalibration('a', &servo_hour, &config.hour_servo.min);
  Serial.println("Calibrating hour max: Position b");
  DoServoCalibration('b', &servo_hour, &config.hour_servo.max);
  Serial.println("Calibrating minute min: Position c");
  DoServoCalibration('c', &servo_minute, &config.minute_servo.min);
  Serial.println("Calibrating minute max: Position d");
  DoServoCalibration('d', &servo_minute, &config.minute_servo.max);
  Serial.println("Calibration done");
  WriteConfigurationToEeprom(config);
}

// The Arduino setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(PIN_BLINKY_LED, OUTPUT);

  // setup setial port
  Serial.begin(115200);

  // Start the I2C interface
  Wire.begin();
  Rtc.Begin();


  // Read configuration from NVM. If it fails, we initialize it to some
  // reasonable default values.
  if (!LoadConfigurationFromEeprom(&config)) {
    config = Configuration::Default();
  }

  // Setup and check the time on the RTC clock
  RtcDateTime time_compiled = RtcDateTime(__DATE__, __TIME__);
  //RtcDateTime time_compiled = RtcDateTime("Mar 05 2021", "12:23:16");

  if (!Rtc.IsDateTimeValid()) {
    if (Rtc.LastError() != 0) {
      // we have a communications error
      // see https://www.arduino.cc/en/Reference/WireEndTransmission for
      // what the number means
      Serial.print("RTC communications error = ");
      Serial.println(Rtc.LastError());
    } else {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      Serial.println("RTC lost confidence in the DateTime!");
      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue

      Rtc.SetDateTime(time_compiled);
      config.DST_when_time_set = IsDST(&time_compiled);
      WriteConfigurationToEeprom(config);
    }
  }

  if (!Rtc.GetIsRunning()) {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  Serial.print("Compile time=");
  Serial.print(uint32_t(time_compiled), DEC);
  Serial.print(" :");
  PrintTime(&time_compiled);

  RtcDateTime time_now = Rtc.GetDateTime();
  Serial.print("    RTC time=");
  Serial.print(uint32_t(time_now), DEC);
  Serial.print(" :");
  PrintTime(&time_now);

  // figure out if we should keep the RTC time or take the compiled time
  if (uint32_t(time_now) < uint32_t(time_compiled)) {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(time_compiled);
    config.DST_when_time_set = IsDST(&time_compiled);
    WriteConfigurationToEeprom(config);
  } else if (uint32_t(time_now) > uint32_t(time_compiled)) {
    Serial.println("RTC is newer than compile time. (this is expected)");
  } else if (uint32_t(time_now) == uint32_t(time_compiled)) {
    Serial.println(
        "RTC is the same as compile time! (not expected but all is fine)");
  }

  if (config.DST_when_time_set = IsDST(&time_compiled)) {
    Serial.println("DST history ok");
  } else {
    Serial.println("DST history mis-match.  Updating DateTime");
    Rtc.SetDateTime(time_compiled);
    config.DST_when_time_set = IsDST(&time_compiled);
    WriteConfigurationToEeprom(config);
  }

  // uncomment this line to force the RTC to take the compiled time
  // Rtc.SetDateTime(time_compiled);

  // disable the square wave output
  Rtc.SetSquareWavePin(DS1307SquareWaveOut_Low);

  // setup servos
  servo_hour.attach(PIN_SERVO_HOUR);
  servo_minute.attach(PIN_SERVO_MINUTE);

  // wake up the MAX72XX from power-saving mode
  display.shutdown(0, false);
  display.setIntensity(0, config.display_brightness);

  up_button.Init(PIN_BUTTON_UP);
  down_button.Init(PIN_BUTTON_DOWN);
  select_button.Init(PIN_BUTTON_SELECT);
}

#define SERVO_SPEED  0.5
float hour_pos_current = 0;
float minute_pos_current = 0;

float ServoVelocityLimit(float current_pos, int goal_pos) {
  float goal_pos_f = (float)goal_pos;
  float diff = fabs(goal_pos_f - current_pos);
  if (diff <= SERVO_SPEED) {
    current_pos = goal_pos_f;
  } else if (goal_pos_f > current_pos) {
    current_pos += SERVO_SPEED;
  } else if (goal_pos_f < current_pos) {
    current_pos -= SERVO_SPEED;
  }
  return current_pos;
}


// Displays the given time on the minute and second hands, using pre-determined
// minimum and maximum servo values derived from our calibration process.
void UpdateServos(const Configuration &config, const RtcDateTime &time) {
  // We display 12 hour time.
  const int minute = time.Minute();
  int hour = time.Hour();
  if (hour > 12) {
    hour -= 12;
  }
  // hour servo homes at noon & midnight
  if (hour == 12) {
    hour = 0;
  }


  // Scale the 720 different possible minutes in a 12 hour span to the servo
  // values.
  int hour_pos_goal = hour * 60 + (int)minute;
  hour_pos_goal =
      map(hour_pos_goal, 0, 720, config.hour_servo.min, config.hour_servo.max);
  hour_pos_goal = constrain(hour_pos_goal, SERVO_MIN, SERVO_MAX);
  hour_pos_current = ServoVelocityLimit(hour_pos_current, hour_pos_goal);
  servo_hour.write(hour_pos_current);

  // Scale the 60 different possible minutes in a 1 hour span to the servo
  // values.
  int minute_pos_goal =
      map(minute, 0, 60, config.minute_servo.min, config.minute_servo.max);
  minute_pos_goal = constrain(minute_pos_goal, SERVO_MIN, SERVO_MAX);
  minute_pos_current = ServoVelocityLimit(minute_pos_current, minute_pos_goal);
  servo_minute.write(minute_pos_current);
}

// State used in the loop, which must be global because of the way arduino
// exposes the loop() function. Rather than declaring this on the stack in main
// we make it global so the values persist between loop iterations.
uint8_t second_old = 0;
uint16_t colon_blink_counter = 0;
bool skipPrint = false;

// Arduino provides a main() function that will repeatedly call loop() in a loop
// as fast as possible. We can fill loop() with code that we want to run
// periodically, and then provide rate limiting so that it runs at a regular
// period.
void loop() {
  // Update the buttons, detecting any new user input.
  up_button.Update();
  down_button.Update();
  select_button.Update();
  light_sensor_update();

  // Holding down the select button triggers the servo calibration routine.
  if (select_button.held()) {
    DoCalibration();
  }

  // Check for display brightness updates, and commit changes to EEPROM.
  const uint8_t led_brightness_old = config.display_brightness;
  RtcDateTime now = Rtc.GetDateTime();
  if (up_button.just_pressed()) {
    //Serial.println("up");
    PrintTime(&now);
    skipPrint = true;
    if (config.display_brightness < LED_BRIGHTNESS_MAX) {
      config.display_brightness++;
    }
  }
  if (down_button.just_pressed()) {
    //Serial.println("down");
    PrintTime(&now);
    skipPrint = true;
    if (config.display_brightness > LED_BRIGHTNESS_MIN) {
      config.display_brightness--;
    }
  }
  if (select_button.just_pressed()) {
    //Serial.println("select");
    PrintTime(&now);
    skipPrint = true;
  }
  if (led_brightness_old != config.display_brightness) {
    //display.setIntensity(0, config.display_brightness);
    WriteConfigurationToEeprom(config);
  }
  display.setIntensity(0, constrain(config.display_brightness + brightness_adjust(), LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX));

  // Check the time, and whenever a second has passed, blink the colon and
  // ensure the time is up to date.
  UpdateServos(config, now);

  if (now.Second() != second_old) {
    DisplayTime(&now);
    if (!skipPrint) {
      PrintTime(&now);
    }
    skipPrint = false;
    colon_blink_counter = COLON_BLINK_TIME;
  }
  second_old = now.Second();

  // Count down some number of loop ticks before turning the colon back off.
  // It's turned back on every second by the code above.
  if (colon_blink_counter > 0) {
    display.setChar(0, 4, ' ', true);
    colon_blink_counter--;
  } else {
    display.setChar(0, 4, ' ', false);
  }
  // Rate limit the loop
  delay(LOOP_PERIOD);
}
