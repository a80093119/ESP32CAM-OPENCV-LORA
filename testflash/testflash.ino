#include "esp_camera.h"
#include "Arduino.h"
 
bool internet_connected = false;
long current_millis;
long last_capture_millis = 0;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
int LED_BUILTIN = 4;


void setup()
{
 Serial.begin(115200);
 pinMode(LED_BUILTIN, OUTPUT);

// if (init_wifi()) { // Connected to WiFi
//   internet_connected = true;
//   Serial.println("Internet connected");
// }

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
 config.pixel_format = PIXFORMAT_GRAYSCALE;
 //init with high specs to pre-allocate larger buffers
 if (psramFound()) {
   config.frame_size = FRAMESIZE_QVGA;
   config.jpeg_quality = 10;
   config.fb_count = 2;
 } else {
   config.frame_size = FRAMESIZE_SVGA;
   config.jpeg_quality = 12;
   config.fb_count = 1;
 }

 // camera init
 esp_err_t err = esp_camera_init(&config);
 if (err != ESP_OK) {
   Serial.printf("Camera init failed with error 0x%x", err);
   return;
 }
}


/********初始化WIFI*********/
//bool init_wifi()
//{
// int connAttempts = 0;
// Serial.println("\r\nConnecting to: " + String(ssid));
// WiFi.begin(ssid, password);
// WiFi.setAutoReconnect(true);
// while (WiFi.status() != WL_CONNECTED ) {
//   delay(500);
//   Serial.print(".");
//   if (connAttempts > 10) return false;
//   connAttempts++;
// }
// return true;
//}



static esp_err_t sendMail2m() {
   //初始化相机并拍照
   Serial.println("Taking picture...");
   camera_fb_t * fb = NULL;
   
   //open flash
   Serial.println("On");
   digitalWrite(LED_BUILTIN, HIGH);
   fb = esp_camera_fb_get();
   //close
   Serial.println("Off");
   digitalWrite(LED_BUILTIN, LOW);
   int height = fb->height;
   int width = fb->width;
//   Serial.print(fb->height); Serial.write(','); // write the height
//   Serial.print(fb->width); Serial.write(','); // write the width
//   Serial.print(fb->len); Serial.write(',');
   int cam_buf[76800];
   for (int i = 0; i < height*width; i++){ // dump the pixels' value
       //Serial.print(fb->buf[i]);
       cam_buf[i] = fb->buf[i];
       Serial.print(cam_buf[i]);
       if (i != height*width -1) Serial.write(',');
       else Serial.println();
   }
     //清空数据
   esp_camera_fb_return(fb);  
}

void loop()
{
   //定时发送
   //当前时间减去上次时间大于20S就执行拍照上传函数 
   delay(30000);
   sendMail2m(); //拍照上传函数，在需要的地方调用即可，这里是定时拍照
   
}
