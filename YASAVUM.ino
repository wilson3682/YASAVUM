/*
 * YASAVUM - Yet another simple Arduino VU meter
 * 
 * Copyright 2016 Thomas Buck <xythobuz@xythobuz.de>
 */

#define CHANNELS 7
#define ROWS 10
#define LINEAR_STEP (1023 / ROWS)
#define ADC_INVALID_VALUE 4242
#define TIMER_RESOLUTION 65536UL

//#define TEST_DISPLAY_PICTURE
//#define TEST_DISPLAY
//#define TEST_VALUES
//#define TEST_MAXIMUM

// ----------------------------------------------------------------
// ------------------------ Configurations ------------------------
// ----------------------------------------------------------------

/*
 * I'd expect the logarithmic scale to better represent
 * the human hearing volume scale. But the linear effect
 * looks much nicer, the logarithm is simply a lit block.
 */
//#define LOGARITHM

/*
 * Used to initialize the timer and also approximately
 * setting the main-loop delay. To run the main-loop
 * at full speed, commment-out the MAIN_DELAY.
 * The main-loop time also influences the dacay behaviour
 * of the MSGEQ7, resulting in lower values at higher speeds.
 */
#define DISPLAY_FPS 100
//#define MAIN_DELAY (1500 / DISPLAY_FPS)
#define TIMER_US (1000000UL / (DISPLAY_FPS * CHANNELS))
#define TIMER_CYCLES ((F_CPU / 2000000UL) * TIMER_US)

/*
 * The following two settings can be completely
 * disabled by commenting-out the define itself.
 * You need to change MAXIMUM_BAR when toggling
 * the MAIN_DELAY.
 */
#define MAXIMUM_BAR 20
//#define ONLY_SHOW_MAXIMUM_OUTLINE
#define NOISE_FILTER 180

/*
 * Pin Configuration for both MSGEQ7. Simply
 * connect the strobe and reset pins together.
 */
#define STROBE_PIN 2
#define RESET_PIN 3
#define ADC_LEFT A0
#define ADC_RIGHT A1

/*
 * Analog volume knob using a poti. Only
 * enabled if both of these are defined.
 */
//#define CALIBRATION_TIME 1000
//#define ADC_CALIBRATE A2

/*
 * Enable Pulse Width Modulating the Segment Pins
 * to control the brightness of the LED display.
 */
#define LED_PWM // uncomment to disable
#define DEFAULT_BRIGHTNESS 10
#define ADC_CONTROL_PWM A2
#define BRIGHTNESS_TIME 100

/*
 * Uuuhhhh, what the fuck?!
 * I really don't understand what's going on here,
 * this _should_ make no difference.
 * But without this, the display looks like shit
 * on low brightness values. Why...? :(
 */
#define USE_ANALOG_WRITE

/*
 * Pins for the 7 different LED bands, connected
 * to both the left and right channel at the
 * same time.
 *
 * DO NOT EDIT THESE PIN ASSIGNMENTS WHEN USING PWM!
 * They are hard-coded to the Timer PWM pins. You
 * can change their order, however...
 */
const uint8_t SEGMENT_PINS[CHANNELS] =
  { 12, 11, 10, 9, 8, 7, 6 };

/*
 * Pins for each LED band of all frequency
 * bands. Best place to switch left and right.
 */
const uint8_t LEFT_PINS[ROWS] =
  { 38, 36, 34, 32, 30, 28, 26, 24, 22, 20 };
const uint8_t RIGHT_PINS[ROWS] =
  { 39, 37, 35, 33, 31, 29, 27, 25, 23, 21 };

// ----------------------------------------------------------------
// ------------------- LED Multiplexing Display -------------------
// ----------------------------------------------------------------

/*
 * Using Timer3 for multiplex timing
 * Using Timer1 for PWM pins 11 & 12
 * Using Timer2 for PWM pins 9 & 10
 * Using Timer4 for PWM pins 6, 7 & 8
 */

// 1 bit for each LED bar, to allow non-consecutive lights
volatile uint16_t displayLeft[CHANNELS] = { 0, 0, 0, 0, 0, 0, 0 };
volatile uint16_t displayRight[CHANNELS] = { 0, 0, 0, 0, 0, 0, 0 };
volatile uint8_t displayPWM[CHANNELS] = { 0, 0, 0, 0, 0, 0, 0 };
volatile uint8_t nextSegment = 0;

void displayInit(void) {
  for (uint8_t i = 0; i < CHANNELS; i++) {
    pinMode(SEGMENT_PINS[i], OUTPUT);
    digitalWrite(SEGMENT_PINS[i], LOW);
  }

  for (uint8_t i = 0; i < ROWS; i++) {
    pinMode(LEFT_PINS[i], OUTPUT);
    digitalWrite(LEFT_PINS[i], LOW);

    pinMode(RIGHT_PINS[i], OUTPUT);
    digitalWrite(RIGHT_PINS[i], LOW);
  }

  noInterrupts();

  /*
   * Dynamically setup Timer3 depending on the FPS
   * compile time option. This has been adapted
   * from the TimerOne Library:
   * http://playground.arduino.cc/Code/Timer1
   */
  TCCR3A = 0;
#if (TIMER_CYCLES < TIMER_RESOLUTION)
  TCCR3B = (1 << WGM33) | (1 << CS30);
  ICR3 = TIMER_CYCLES;
#elif (TIMER_CYCLES < (TIMER_RESOLUTION * 8))
  TCCR3B = (1 << WGM33) | (1 << CS31);
  ICR3 = TIMER_CYCLES / 8;
#elif (TIMER_CYCLES < (TIMER_RESOLUTION * 64))
  TCCR3B = (1 << WGM33) | (1 << CS30) | (1 << CS31);
  ICR3 = TIMER_CYCLES / 64;
#elif (TIMER_CYCLES < (TIMER_RESOLUTION * 256))
  TCCR3B = (1 << WGM33) | (1 << CS32);
  ICR3 = TIMER_CYCLES / 256;
#elif (TIMER_CYCLES < (TIMER_RESOLUTION * 1024))
  TCCR3B = (1 << WGM33) | (1 << CS30) | (1 << CS32);
  ICR3 = TIMER_CYCLES / 1024;
#else
  TCCR3B = (1 << WGM33) | (1 << CS30) | (1 << CS32);
  ICR3 = TIMER_RESOLUTION - 1;
#endif
  TIMSK3 = (1 << TOIE3);

  /*
   * Setup the other required timers, all in the
   * 8bit Fast PWM mode with Prescaler 1 and
   * corresponding output pins enabled.
   */
#ifdef LED_PWM
  // Setup Timer1
  TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM12) | (1 << CS10);
  OCR1A = DEFAULT_BRIGHTNESS;
  OCR1B = DEFAULT_BRIGHTNESS;

  // Setup Timer2 (8-bit, different registers!)
  TCCR2A = (1 << COM2A1) | (1 << COM2B1)
          | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS20);
  OCR2A = DEFAULT_BRIGHTNESS;
  OCR2B = DEFAULT_BRIGHTNESS;

  // Setup Timer4
  TCCR4A = (1 << WGM40) | (1 << COM4A1)
          | (1 << COM4B1) | (1 << COM4C1);
  TCCR4B = (1 << WGM42) | (1 << CS40);
  OCR4A = DEFAULT_BRIGHTNESS;
  OCR4B = DEFAULT_BRIGHTNESS;
  OCR4C = DEFAULT_BRIGHTNESS;
#endif

  interrupts();
}

void setData(uint16_t l, uint16_t r, uint8_t s, uint8_t i) {
  // avoid displaying corrupt / invalid data
  noInterrupts();
  displayLeft[i] = l;
  displayRight[i] = r;
  displayPWM[i] = s;
  interrupts();
}

#ifdef LED_PWM
void setPWMValue(uint8_t pin, uint8_t val) {
  if (pin == 6) {
    OCR4A = val;
  } else if (pin == 7) {
    OCR4B = val;
  } else if (pin == 8) {
    OCR4C = val;
  } else if (pin == 9) {
    OCR2B = val;
  } else if (pin == 10) {
    OCR2A = val;
  } else if (pin == 11) {
    OCR1A = val;
  } else {
    OCR1B = val;
  }
}
#endif

ISR(TIMER3_OVF_vect) {
  uint8_t lastSegment =
      (nextSegment > 0) ? nextSegment - 1 : CHANNELS - 1;

#ifdef LED_PWM
#ifdef USE_ANALOG_WRITE
  analogWrite(SEGMENT_PINS[lastSegment], 0);
#else
  setPWMValue(SEGMENT_PINS[lastSegment], 0);
#endif
#else
  digitalWrite(SEGMENT_PINS[lastSegment], LOW);
#endif

  for (uint8_t i = 0; i < ROWS; i++) {
    digitalWrite(LEFT_PINS[i],
        (displayLeft[nextSegment] & (1 << i)) ? HIGH : LOW);

    digitalWrite(RIGHT_PINS[i],
        (displayRight[nextSegment] & (1 << i)) ? HIGH : LOW);
  }

#ifdef LED_PWM
#ifdef USE_ANALOG_WRITE
  analogWrite(SEGMENT_PINS[nextSegment], displayPWM[nextSegment]);
#else
  setPWMValue(SEGMENT_PINS[nextSegment], displayPWM[nextSegment]);
#endif
#else
  digitalWrite(SEGMENT_PINS[nextSegment], HIGH);
#endif

  nextSegment =
      (nextSegment < (CHANNELS - 1)) ? nextSegment + 1 : 0;
}

// ----------------------------------------------------------------
// ------------------- MSGEQ7 Graphic Equalizer -------------------
// ----------------------------------------------------------------

/*
 * Very simple implementation with exact timings
 * taken from the MSGEQ7 datasheet.
 */
#define Trs 72
#define Ts 19
#define To 36

uint16_t dataLeft[CHANNELS] = { 0, 0, 0, 0, 0, 0, 0 };
uint16_t dataRight[CHANNELS] = { 0, 0, 0, 0, 0, 0, 0 };

void equalizerInit(void) {
  pinMode(STROBE_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  // Disable MSGEQ7
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(STROBE_PIN, LOW);
}

void equalizerRead(void) {
  // Enable MSGEQ7
  digitalWrite(RESET_PIN, LOW);
  delayMicroseconds(Trs);

  for (uint8_t i = 0; i < CHANNELS; i++) {
    // Strobe the next channel
    digitalWrite(STROBE_PIN, HIGH);
    delayMicroseconds(Ts);
    digitalWrite(STROBE_PIN, LOW);
    delayMicroseconds(To);

    // Read data for both channels
    dataLeft[i] = analogRead(ADC_LEFT);
    dataRight[i] = analogRead(ADC_RIGHT);

    // Cheap noise filter, just remove small values...
#ifdef NOISE_FILTER
    if (dataLeft[i] < NOISE_FILTER) {
      dataLeft[i] = 0;
    }

    if (dataRight[i] < NOISE_FILTER) {
      dataRight[i] = 0;
    }
#endif
  }

  // Disable MSGEQ7 again
  digitalWrite(RESET_PIN, HIGH);
}

// ----------------------------------------------------------------
// ------------------------- Main Program -------------------------
// ----------------------------------------------------------------

uint8_t brightness = DEFAULT_BRIGHTNESS;

// logarithmic scale
uint16_t draw(uint16_t value) {
  uint16_t ret = 0;

#ifdef LOGARITHM
  for (uint8_t j = 0; j < ROWS; j++) {
    if (value >= (1 << j)) {
      ret |= (1 << j);
    }
  }
#else
  for (uint8_t j = 0; j < ROWS; j++) {
    if (value >= ((j + 1) * LINEAR_STEP)) {
      ret |= (1 << j);
    }
  }
#endif

  return ret;
}

#if defined(ADC_CALIBRATE) && defined(CALIBRATION_TIME)
void offset(void) {
  static uint16_t multiplier = ADC_INVALID_VALUE;
  static uint16_t count = 0;

  if ((count >= CALIBRATION_TIME)
      || (multiplier == ADC_INVALID_VALUE)) {
    count = 1;
    multiplier = (analogRead(ADC_CALIBRATE) + 100) / 10;
  } else {
    count++;
  }

  for (uint8_t i = 0; i < CHANNELS; i++) {
    displayLeft[i] = displayLeft[i] * multiplier / 10;
    if (displayLeft[i] > 1023) {
      displayLeft[i] = 1023;
    }

    displayRight[i] = displayRight[i] * multiplier / 10;
    if (displayRight[i] > 1023) {
      displayRight[i] = 1023;
    }
  }
}
#endif

#ifdef ADC_CONTROL_PWM
void readBrightness(void) {
  static uint16_t count = BRIGHTNESS_TIME;
  if (count >= BRIGHTNESS_TIME) {
    count = 1;
    brightness = analogRead(ADC_CONTROL_PWM) >> 2;
  } else {
    count++;
  }
}
#endif

#ifdef MAXIMUM_BAR
/*
 * Store maxima in left/rightMax, and a decayTimer in
 * left/rightDecay. If there*s a new maximum, reset
 * the decay timer. If the decay has reached zero,
 * reset it and decrease the max a step.
 * Returns bit of maximum stripe in arguments.
 */

uint16_t leftMax[CHANNELS] = { 0, 0, 0, 0, 0, 0, 0 };
uint16_t rightMax[CHANNELS] = { 0, 0, 0, 0, 0, 0, 0 };
uint16_t leftDecay[CHANNELS] = { MAXIMUM_BAR,
    MAXIMUM_BAR, MAXIMUM_BAR, MAXIMUM_BAR,
    MAXIMUM_BAR, MAXIMUM_BAR, MAXIMUM_BAR };
uint16_t rightDecay[CHANNELS] = { MAXIMUM_BAR,
    MAXIMUM_BAR, MAXIMUM_BAR, MAXIMUM_BAR,
    MAXIMUM_BAR, MAXIMUM_BAR, MAXIMUM_BAR };

void maximum(uint16_t *left, uint16_t *right, uint8_t i) {
  if (leftDecay[i] > 0) {
    leftDecay[i]--;
  }

  if (rightDecay[i] > 0) {
    rightDecay[i]--;
  }

  if (leftDecay[i] == 0) {
    leftDecay[i] = MAXIMUM_BAR;
    if (leftMax[i] >= LINEAR_STEP) {
      leftMax[i] -= LINEAR_STEP;
    } else {
      leftMax[i] = 0;
    }
    
#ifdef TEST_MAXIMUM
    Serial.print("Left[");
    Serial.print(i);
    Serial.print("] has decayed: ");
    Serial.println(leftMax[i]);
#endif
  }

  if (rightDecay[i] == 0) {
    rightDecay[i] = MAXIMUM_BAR;
    if (rightMax[i] >= LINEAR_STEP) {
      rightMax[i] -= LINEAR_STEP;
    } else {
      rightMax[i] = 0;
    }
    
#ifdef TEST_MAXIMUM
    Serial.print("Right[");
    Serial.print(i);
    Serial.print("] has decayed: ");
    Serial.println(rightMax[i]);
#endif
  }

  if (*left > leftMax[i]) {
#ifdef TEST_MAXIMUM
    Serial.print("New Left[");
    Serial.print(i);
    Serial.print("] Max: ");
    Serial.print(*left);
    Serial.print(" > ");
    Serial.println(leftMax[i]);
#endif

    leftMax[i] = *left;
    leftDecay[i] = MAXIMUM_BAR;
  }

  if (*right > rightMax[i]) {
#ifdef TEST_MAXIMUM
    Serial.print("New Right[");
    Serial.print(i);
    Serial.print("] Max: ");
    Serial.print(*right);
    Serial.print(" > ");
    Serial.println(rightMax[i]);
#endif

    rightMax[i] = *right;
    rightDecay[i] = MAXIMUM_BAR;
  }

#ifdef TEST_MAXIMUM
  Serial.print("Left[");
  Serial.print(i);
  Serial.print("] Max Bar: ");
  Serial.println(*left / LINEAR_STEP);
  Serial.print("Right[");
  Serial.print(i);
  Serial.print("] Max Bar: ");
  Serial.println(*right / LINEAR_STEP);
#endif

  if (leftMax[i] >= LINEAR_STEP) {
    *left = 1 << ((leftMax[i] / LINEAR_STEP) - 1);
  } else {
    *left = 0;
  }

  if (leftMax[i] >= LINEAR_STEP) {
    *right = 1 << ((rightMax[i] / LINEAR_STEP) - 1);
  } else {
    *right = 0;
  }
}
#endif // MAXIMUM_BAR

void setup(void) {
  displayInit();
  equalizerInit();

#if defined(TEST_VALUES) || defined(TEST_MAXIMUM)
  Serial.begin(115200);
#endif
}

#ifdef TEST_DISPLAY_PICTURE
uint16_t testValues[14] = { 1, 7, 15, 63, 255, 511, 1023, 1023, 511, 255, 127, 31, 15, 7 };
#endif

void loop(void) {
#ifdef TEST_DISPLAY_PICTURE
  for (uint8_t i = 0; i < 7; i++) {
    setData(testValues[i], testValues[7 + i], DEFAULT_BRIGHTNESS, i);
  }

  uint16_t tmp = testValues[13];
  for (int i = 12; i >= 0; i--) {
    testValues[i + 1] = testValues[i];
  }
  testValues[0] = tmp;

  delay(50);
#elif defined(TEST_DISPLAY)
  for (uint8_t i = 0; i < 7; i++) {
    for (uint8_t j = 0; j < 10; j++) {
      setData((1 << j), (1 << j), DEFAULT_BRIGHTNESS, i);
      delay(50);
    }
  }
#else
  equalizerRead();

#if defined(ADC_CALIBRATE) && defined(CALIBRATION_TIME)
  offset();
#endif

#ifdef ADC_CONTROL_PWM
  readBrightness();
#endif

#ifdef TEST_VALUES
  for (uint8_t i = 0; i < CHANNELS; i++) {
    Serial.print(dataLeft[i]);
    if (i < (CHANNELS - 1)) {
      Serial.print(" ");
    } else {
      Serial.println();
    }
  }
  for (uint8_t i = 0; i < CHANNELS; i++) {
    Serial.print(dataRight[i]);
    if (i < (CHANNELS - 1)) {
      Serial.print(" ");
    } else {
      Serial.println();
    }
  }
  Serial.println();
#endif

  for (uint8_t i = 0; i < CHANNELS; i++) {
    uint16_t left = draw(dataLeft[i]);
    uint16_t right = draw(dataRight[i]);

#ifdef MAXIMUM_BAR
    uint16_t lm = dataLeft[i], rm = dataRight[i];
    maximum(&lm, &rm, i);

    left |= lm;
    right |= rm;
    
#ifdef ONLY_SHOW_MAXIMUM_OUTLINE
    setData(lm, rm, brightness, i);
#else
    setData(left, right, brightness, i);
#endif
#else
    setData(left, right, brightness, i);
#endif
  }

#ifdef MAIN_DELAY
  delay(MAIN_DELAY);
#endif

#endif
}

