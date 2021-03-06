#include "FastLED.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

FASTLED_USING_NAMESPACE


//#define GENERAL_DEBUG
//#define MPU_DEBUG
#define ROOMBA_OUTPUT

#define INTERRUPT_PIN 2
#define DATA_PIN    11
#define CLK_PIN   13
#define CHEST_NUM_LEDS 59
#define WRIST_NUM_LEDS 16
#define TOTAL_NUM_LEDS 165
#define LED_TYPE    LPD8806
#define COLOR_ORDER BRG
CRGB unique_leds[CHEST_NUM_LEDS];
CRGB actual_leds[TOTAL_NUM_LEDS];

uint16_t generator_timer = 200;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define BLUR_RATE 5

int8_t left_index = 0;
int8_t right_index = 0;

uint32_t shift_update_time = 0;
uint32_t last_hue_time = 0;
uint32_t last_effect_time = 0;
uint32_t mode_swap_timer = 0;

uint32_t tap_time = 0;
uint8_t tap_count = 0;
bool tap_counted = false;

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

#define JACKET_OFF 0
#define JACKET_ON 1
#define JACKET_MOTION 2
uint8_t jacket_mode = JACKET_MOTION;

#define LEDS_CW 0
#define LEDS_CCW 1
#define LEDS_UP 2
#define LEDS_DOWN 3
uint8_t led_direction = LEDS_UP;

#define ROOMBA_IDLE 0
#define ROOMBA_OFF 1
#define ROOMBA_START 2
#define ROOMBA_ON 128
uint8_t roomba_mode = ROOMBA_OFF;
uint32_t roomba_bootup_timer = 0;

//status stuff for cpu usage
uint32_t fps_time = 0;
uint32_t idle_microseconds = 0;
uint8_t cpu_usage = 0;

int16_t shift_update_speed = 100;
bool left_off = true;
bool right_off = true;
uint32_t fade_timer = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion qq;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(57600);

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(1827);
  mpu.setYAccelOffset(2960);
  mpu.setZAccelOffset(1280);
  mpu.setXGyroOffset(95);
  mpu.setYGyroOffset(-72);
  mpu.setZGyroOffset(38);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    // Serial.println(F(")"));
  }

  FastLED.addLeds<LED_TYPE, DATA_PIN, CLK_PIN, COLOR_ORDER>(actual_leds, TOTAL_NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(255);

}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  uint32_t idle_start_timer = 0;
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

    // other program behavior stuff here
    if (micros() - fps_time > 1000000) {
      cpu_usage = 100 - (idle_microseconds / 10000);

#ifdef GENERAL_DEBUG
      Serial.println(cpu_usage);
#endif


#ifdef ROOMBA_OUTPUT
      if (roomba_mode == ROOMBA_OFF)  roomba_move(0, 0); //stop motors but dont shut off
#endif


      idle_microseconds = 0;
      fps_time = micros();
    }


    //slowly rotate through demo modes
    //  if (millis() - mode_swap_timer > 10000) {
    //   led_direction++;
    //    if (led_direction > 3) led_direction = 0;
    //    mode_swap_timer = millis();
    //  }

    //force scrolling other side quickly
    if (millis() - shift_update_time > shift_update_speed) {

      if (left_off == false) {
        if (led_direction == LEDS_UP || led_direction == LEDS_CW) {
          left_index++;
          if (left_index >= CHEST_NUM_LEDS) left_index = 0;
        } else {
          left_index--;
          if (left_index < 0) left_index = CHEST_NUM_LEDS - 1;
        }
      }
      if ( right_off == false) {
        if (led_direction == LEDS_DOWN || led_direction == LEDS_CW) {
          right_index++;
          if (right_index >= CHEST_NUM_LEDS) right_index = 0;
        } else  {
          right_index--;
          if (right_index < 0) right_index = CHEST_NUM_LEDS - 1;
        }

      }
      shift_update_time = millis();
    }
    //spawn more dots
    if (millis() - fade_timer > 200 ) {
      fadeToBlackBy( unique_leds, CHEST_NUM_LEDS, 10);
      fade_timer = millis();
    }
    if (millis() - last_effect_time > generator_timer && jacket_mode != JACKET_OFF && generator_timer < 2000) {

      add_pixel();
      last_effect_time = millis();
    }

    //alter hue slowly over time
    if (millis() - last_hue_time > 1000) {
      gHue++;
      last_hue_time = millis();
    }

    //dont count first cycle, its not idle time, its required
    if (idle_start_timer == 0) {
      idle_start_timer = micros();
    }

  }

  idle_microseconds = idle_microseconds + (micros() - idle_start_timer);

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

#ifdef GENERAL_DEBUG
    Serial.println(F("FIFO overflow!"));
#endif

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&qq, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &qq);
    mpu.dmpGetYawPitchRoll(ypr, &qq, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);


    float yangle = ypr[1] * 180 / M_PI;
    float zangle = ypr[2] * 180 / M_PI;

    if (yangle > 0) led_direction = LEDS_DOWN;
    else led_direction = LEDS_UP ;

#define deadzone 10.0
#define maxspeed 50 //minimum of 50ms between shifts, sets maxspeed

    //calculate speed based on y angle
    shift_update_speed = max(200 - 3 * (abs(yangle) - deadzone), maxspeed);
    //main up down

    if (abs(yangle) < deadzone) {
      left_off = true;
      right_off = true;
    } else {
      left_off = false;
      right_off = false;
    }


    //calculate speed based on z angle, use whichever speed is greater (smaller time)

    shift_update_speed = min(shift_update_speed, max(200 - 3 * (abs(zangle) - 2 * deadzone), maxspeed));

    //add in the cc or ccw tweaking
    if (abs(zangle) > 2 * deadzone) {
      if (zangle > 0) {
        led_direction = LEDS_CW;
        left_off = false;
      }
      else {
        led_direction = LEDS_CCW ;
        right_off = false;
      }
    }

    //some kind of slow dim thing, figure it later
    uint32_t current_acceleration = abs(aaReal.z) + abs( aaReal.y ) + abs(aaReal.x );

#ifdef GENERAL_DEBUG
    Serial.println();
    Serial.println(current_acceleration);
#endif

    if (current_acceleration > 10000 && tap_counted == false && millis() - tap_time > 300) {
      tap_count++;
      tap_time = millis();
      tap_counted = true;
      set_wrist_color( CRGB::White );  //white pop for hit detect
      if (jacket_mode == JACKET_MOTION) jumpstart(16);
    }
    if (current_acceleration < 4000) {
      tap_counted = false;
    }

    if (tap_count > 0) {
      if (millis() - tap_time > 1000) {

#ifdef GENERAL_DEBUG
        Serial.print("TAP! ");
        Serial.println(tap_count);
#endif

        if (tap_count == 1) {
          set_wrist_color(CRGB::Purple);
          gHue = 192;
          if (jacket_mode != JACKET_OFF) jumpstart(8);
        }
        else if (tap_count == 2) {

          if (jacket_mode == JACKET_OFF) jacket_mode = JACKET_MOTION;
          set_wrist_color(CRGB::Blue);
          gHue = 160;
          jumpstart(8);
        }
        else if (tap_count == 3) { //roomba drive mode
          jacket_mode = JACKET_ON;
          roomba_mode = ROOMBA_START; //this will initiate the roomba bootup sequence and then kick into ROOMBA_ON
          set_wrist_color(CRGB::Lime);
          gHue = 96;
          jumpstart(8);
        } else  if (tap_count == 4) { //roomba spot clean
          jacket_mode = JACKET_MOTION;
          roomba_mode = ROOMBA_IDLE;//dont send the off command, let it keep cleaning
          byte specialmode[2] = {136, 2};  //spot clean demo
          Serial.write(specialmode, 2);
          set_wrist_color( CRGB::Yellow);
          gHue = 64;
          jumpstart(8);
        } else   if (tap_count == 5) {
          jacket_mode = JACKET_OFF;
          roomba_mode = ROOMBA_OFF;
          for (uint8_t i = 0; i < CHEST_NUM_LEDS; i++)  unique_leds[i] = CRGB(0, 0, 0);
          set_wrist_color( CRGB::Red );
          gHue = 230;
        }
        tap_count = 0;
      }
    }

    wakeup();

#ifdef MPU_DEBUG
    Serial.print(current_acceleration);
    Serial.print("\t");
    Serial.print(yangle);
    Serial.print("\t");
    Serial.print(zangle);
    Serial.print("\t");
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

    //roomba motion calculations
    if (roomba_mode == ROOMBA_OFF || roomba_mode == ROOMBA_ON )  roomba_move(yangle, zangle);

    //   if (tap_counted) current_brightness = 255;

    if (jacket_mode == JACKET_ON)    {

      generator_timer = 200;
    }
    else if (jacket_mode == JACKET_MOTION) {

      if (millis() - tap_time > 2000) {
        if (generator_timer < 4000) generator_timer += 100;
      }
    }

    //update LEDS at 100hz, use the interrupt from the MPU as the timer
    led_render_engine();
  }
}

void jumpstart(uint8_t level) {
  for (uint8_t i = 0; i < level; i++) {
    fadeToBlackBy( unique_leds, CHEST_NUM_LEDS, 10);
    add_pixel();
  }
}

void add_pixel() {
  int pos = random16(CHEST_NUM_LEDS);
  unique_leds[pos] += CHSV( gHue + random8(64), 200, 255);

}

inline void led_render_engine() {
  uint8_t led2counter = 0;
  for (uint8_t i = 0; i < WRIST_NUM_LEDS; i++) blend_and_output((i +  left_index)  % CHEST_NUM_LEDS, led2counter++);
  for (uint8_t i = 0; i < CHEST_NUM_LEDS; i++) blend_and_output((i +  left_index)  % CHEST_NUM_LEDS, led2counter++);
  for (uint8_t i = 0; i < CHEST_NUM_LEDS; i++) blend_and_output((i +  right_index) % CHEST_NUM_LEDS, led2counter++);
  for (uint8_t i = 0; i < WRIST_NUM_LEDS; i++) blend_and_output((i +  right_index) % CHEST_NUM_LEDS, led2counter++);
  FastLED.show();
}

inline void set_wrist_color(CRGB color) {
  for (uint8_t i = 0; i < WRIST_NUM_LEDS; i++)   actual_leds[i] = actual_leds[i + 150 - 16] = color;
}

inline void blend_and_output( uint8_t in, uint8_t out) {
  actual_leds[out] = blend(actual_leds[out], unique_leds[in], BLUR_RATE);
}

inline void roomba_move(int16_t yangle, int16_t zangle) {
  int16_t velocity = 0;
  int16_t turning = 0x8000; //special case for straight

  if ( yangle > 20) {
    velocity = map(yangle, 20, 40, 0, -500);
  } else if ( yangle < -20) {
    velocity = map(yangle, -20, -40, 0, 500);
  }

  if ( zangle > 40) {
    turning = 0xFFFF;//special case for turn in place
    velocity = 500;
  } else if ( zangle < -40) {
    turning = 0x0001;//special case for turn in place
    velocity = 500;
  } else if ( zangle > 20) {
    turning =  map(zangle, 20, 40, -500, -1);
    if (turning > -1) turning = -1;
  } else if ( zangle < -20) {
    turning = map(zangle, -20, -40, 500, 1);
    if (turning < 1) turning = 1;
  }

  velocity = constrain(velocity, -500, 500);

  Serial.write(137);
  Serial.write((byte)((velocity >> 8) & 0xFF));
  Serial.write((byte)((velocity) & 0xFF));
  Serial.write((byte)((turning >> 8) & 0xFF));
  Serial.write((byte)((turning) & 0xFF));
}

inline void wakeup() {  //no delay wakeup
  //ROOMBA_START == 1
  //ROOMBA_ON == 128
  if (roomba_mode == ROOMBA_START) {
    pinMode(5, INPUT);
    digitalWrite(5, 0);
    pinMode(5, OUTPUT);
    digitalWrite(5, 0);
    roomba_bootup_timer = millis();
    roomba_mode++;
  }
  else if ((roomba_mode == (ROOMBA_START + 1)) && (( millis() - roomba_bootup_timer ) > 1000)) {
    pinMode(5, INPUT);
    roomba_bootup_timer = millis();
    roomba_mode++;
  }
  else if ((roomba_mode == (ROOMBA_START + 2)) && ( (millis() - roomba_bootup_timer) > 20)) {
    Serial.write(128);
    roomba_bootup_timer = millis();
    roomba_mode++;
  }
  else if ((roomba_mode == ( ROOMBA_START + 3)) && ((millis() - roomba_bootup_timer) > 20)) {
    Serial.write(128);
    roomba_bootup_timer = millis();
    roomba_mode++;
  }
  else if ((roomba_mode == (ROOMBA_START + 4)) && (( millis() - roomba_bootup_timer) > 20)) {
    Serial.write(130);
    roomba_bootup_timer = millis();
    roomba_mode++;
  }
  else if ((roomba_mode == (ROOMBA_START + 5)) && ((millis() - roomba_bootup_timer) > 20)) {
    roomba_mode = ROOMBA_ON;
  }

#ifdef debug
  Serial.println("Roomba Initialized (start, command)");
#endif

}
