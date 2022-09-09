#include "esp_camera.h"
#include <WiFi.h>

#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);

#include <Wire.h>

//WiFi
const char* ssid = "Casa_WiFi_2g";
const char* password = "P@ssw0rd@2020";

const char *ssid_recovery = "Esp_WiFi";
const char *password_recovery = "P@ssw0rd";

//Servo Generico
const char* topic_servo1 = "servo/servo1";        // Servo 1
const char* topic_servo2 = "servo/servo2";        // Servo 2


//IP Broker
const char* mqtt_server = "10.0.2.43";
//NomeClient
const char* clientName = "ESP32_n1";
const int pinLed1 = 0;
const int pinLed2 = 2;
long lastMsg = 0;
String msgIn;
int value = 0;

// Select camera model
#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"


void startCameraServer();

void setup() {
  
  //Led
  pinMode(pinLed1, OUTPUT);
  pinMode(pinLed2, OUTPUT);

  Serial.begin(115200);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connesso");

  WiFi.softAP(ssid_recovery, password_recovery);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP : ");
  Serial.println(myIP);
  

  startCameraServer();

  Serial.print("Camera Pronta! Usa 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void callback(char* topic, byte* payload, unsigned int length) {
  msgIn = "";
  Serial.print("Messaggio arrivato [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    msgIn = msgIn + ((char)payload[i]);
  }
  Serial.println(msgIn);
  ControlsByMessage((String)topic, msgIn);
}

void ControlsByMessage(String tpc, String msg)
{
    Serial.println(tpc);
    Serial.println(msg);
  if (tpc == "L1")
  {  
    if (msg == "ON")
    {
      digitalWrite(0, HIGH);
    }
    else if (msg == "OFF")
    {
      digitalWrite(0, LOW);
    }
  }
  if (tpc == "L2")
  {
    if (msg == "ON")
    {
      digitalWrite(1, HIGH);
    }
    else if (msg == "OFF")
    {
      digitalWrite(1, LOW);
    }
  }
}

void reconnect() {
  // Loop fino alla riconnessione
  while (!client.connected()) {
    Serial.print("Connetto a MQTT...");
    // tenta la connessione
    if (client.connect(clientName)) {
      Serial.println("connected");
      // sottoscrizione ai Topic
      client.subscribe("L1");
      client.subscribe("L2");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" connessione in 5 seconds");
      // attendi 5 secondi tra i tentativi di riconnessione
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
