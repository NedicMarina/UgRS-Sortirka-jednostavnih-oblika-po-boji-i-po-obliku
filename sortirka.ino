#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <ESP32Servo.h>

#include "board_config.h"


#define SERVO_PIN 14

// IR senzori
#define IR1_PIN 3
#define IR2_PIN 13

// L298N
#define MOTOR_IN1 15
#define MOTOR_IN2 2
#define MOTOR_ENA 4



// PWM raspon je 0 - 255
#define MOTOR_SPEED 180


// IR SENZORI

#define IR_ACTIVE_STATE LOW
#define IR_IDLE_STATE HIGH


// SERVO

#define SERVO_CENTER_ANGLE 90
#define SERVO_SORT_ANGLE 30
#define SERVO_SORT_HOLD_MS 2000

// PWM KANALI

#define MOTOR_PWM_CHANNEL 4

// WIFI
const char *ssid = "*******";
const char *password = "*******";

IPAddress localIP(192, 168, 2, 222);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 2, 1);


// GLOBALNE VARIJABLE ZA app_httpd.cpp
volatile bool ir1Event = false;
volatile int pendingDecision = -1;

 
// STANJE IR SENZORA
int previousIR1 = IR_IDLE_STATE;
int previousIR2 = IR_IDLE_STATE;


// FUNKCIJE IZ app_httpd.cpp

void startCameraServer();


// SERVO FUNKCIJA

Servo sortingServo;

void servoWriteAngleLocal(int angle)
{
  sortingServo.write(constrain(angle, 0, 180));
}


// SORTIRANJE CILJNOG OBJEKTA
static void sortTargetObject()
{
  servoWriteAngleLocal(SERVO_SORT_ANGLE);

  delay(SERVO_SORT_HOLD_MS);

  servoWriteAngleLocal(SERVO_CENTER_ANGLE);
}


void setup()
{
  Serial.begin(115200);

  Serial.setDebugOutput(false);

  Serial.println();
  Serial.println("=================================");
  Serial.println("ESP32-CAM SORTING SYSTEM");
  Serial.println("=================================");


  // IR SENZORI
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);


  // MOTOR - SMJER

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);



  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;

  config.frame_size = FRAMESIZE_UXGA;

  config.pixel_format = PIXFORMAT_JPEG;

  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  config.fb_location = CAMERA_FB_IN_PSRAM;

  config.jpeg_quality = 12;

  config.fb_count = 1;



  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    if (psramFound())
    {
      config.jpeg_quality = 10;
      config.fb_count = 2;

      config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
      config.frame_size = FRAMESIZE_SVGA;

      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }
  else
  {
    config.frame_size = FRAMESIZE_240X240;

#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }


#if defined(CAMERA_MODEL_ESP_EYE)

  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

#endif


  // POKRETANJE KAMERE
  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK)
  {
    Serial.printf(
      "Camera init failed with error 0x%x\n",
      err
    );

    return;
  }


  sensor_t *s = esp_camera_sensor_get();

  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);

    s->set_brightness(s, 1);

    s->set_saturation(s, -2);
  }


  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }


#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)

  s->set_vflip(s, 1);

  s->set_hmirror(s, 1);

#endif


#if defined(CAMERA_MODEL_ESP32S3_EYE)

  s->set_vflip(s, 1);

#endif


  Serial.println("Camera initialized.");


 
  bool motorPWMok = ledcAttachChannel(
    MOTOR_ENA,
    5000,
    8,
    MOTOR_PWM_CHANNEL
  );


  if (motorPWMok)
  {
    Serial.println("Motor PWM initialized.");
  }
  else
  {
    Serial.println("ERROR: Motor PWM initialization failed!");
  }


  // Motor mora biti ugašen pri pokretanju
  ledcWrite(MOTOR_ENA, 0);

  digitalWrite(MOTOR_IN1, LOW);

  digitalWrite(MOTOR_IN2, LOW);

  // SERVO PWM

  ESP32PWM::allocateTimer(3);
  sortingServo.setPeriodHertz(50);
  int servoChannel = sortingServo.attach(SERVO_PIN, 500, 2400);


  if (servoChannel > 0)
  {
    Serial.println("Servo initialized.");
  }
  else
  {
    Serial.println("ERROR: Servo PWM initialization failed!");
  }


  // Servo u sredinu
  servoWriteAngleLocal(SERVO_CENTER_ANGLE);


  Serial.println("Motor is OFF.");

  Serial.println("Servo is CENTERED.");


  // POČETNO STANJE SENZORA

  previousIR1 = digitalRead(IR1_PIN);

  previousIR2 = digitalRead(IR2_PIN);


  WiFi.mode(WIFI_STA);

  if (!WiFi.config(localIP, gateway, subnet, dns))
  {
    Serial.println("ERROR: Static IP configuration failed!");
  }

  WiFi.begin(
    ssid,
    password
  );

  WiFi.setSleep(false);


  Serial.print("WiFi connecting");


  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);

    Serial.print(".");
  }


  Serial.println();

  Serial.println("WiFi connected");



  startCameraServer();


  // ISPIS IP ADRESE

  Serial.println();

  Serial.println("=================================");

  Serial.print("Camera Ready! Use 'http://");

  Serial.print(WiFi.localIP());

  Serial.println("' to connect");


  Serial.println();


  Serial.print("Motor ON:     http://");

  Serial.print(WiFi.localIP());

  Serial.println("/motor_on");


  Serial.print("Motor OFF:    http://");

  Serial.print(WiFi.localIP());

  Serial.println("/motor_off");


  Serial.print("Servo LEFT:   http://");

  Serial.print(WiFi.localIP());

  Serial.println("/servo_left");


  Serial.print("Servo CENTER: http://");

  Serial.print(WiFi.localIP());

  Serial.println("/servo_center");


  Serial.print("Servo RIGHT:  http://");

  Serial.print(WiFi.localIP());

  Serial.println("/servo_right");


  Serial.print("Sensors:      http://");

  Serial.print(WiFi.localIP());

  Serial.println("/sensors");


  Serial.println("=================================");
}



void loop()
{
  int currentIR1 = digitalRead(IR1_PIN);

  int currentIR2 = digitalRead(IR2_PIN);



  if (
    previousIR1 == IR_IDLE_STATE &&
    currentIR1 == IR_ACTIVE_STATE
  )
  {
    ir1Event = true;

    Serial.println(
      "IR1 triggered: object at camera"
    );
  }



  if (
    previousIR2 == IR_IDLE_STATE &&
    currentIR2 == IR_ACTIVE_STATE
  )
  {
    int decision = pendingDecision;

    pendingDecision = -1;


    Serial.print(
      "IR2 triggered: decision = "
    );

    Serial.println(decision);


    if (decision == 1)
    {
      Serial.println(
        "Target object: sorting with servo"
      );

      sortTargetObject();
    }
    else
    {
      Serial.println(
        "Not target: servo stays centered"
      );

      servoWriteAngleLocal(
        SERVO_CENTER_ANGLE
      );
    }
  }


  previousIR1 = currentIR1;

  previousIR2 = currentIR2;


  delay(10);
}
