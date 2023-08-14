#undef EPS // specreg.h defines EPS which interfere with opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192

#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_err.h>
#include "esp_wifi.h"
#include <esp_timer.h>
#include <driver/uart.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_freertos_hooks.h>
#include <iostream>
#include <cmath>
#include <map>

#include "system.h"
#include "app_screen.h"
#include "app_camera.h"

#include "iot_lvgl.h"

#include <string.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

using namespace cv;
using namespace std;

extern "C"
{
    void app_main(void);
}


// #define WIFI_SSID       "ChangChiaLen"
// #define WIFI_PASSWORD   "ab230304"

// #define EXAMPLE_WIFI_SSID             "ChangChiaLen"
// #define EXAMPLE_WIFI_PASS             "ab230304"
// #define EXAMPLE_MAXIMUM_RETRY         10


// #define EXAMPLE_STATIC_IP_ADDR        "172.20.10.2"
// #define EXAMPLE_STATIC_NETMASK_ADDR   "255.255.255.0"
// #define EXAMPLE_STATIC_GW_ADDR        "0.0.0.0"
/* FreeRTOS event group to signal when we are connected*/
// static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define EXAMPLE_ESP_WIFI_SSID      "ChangChiaLun"
#define EXAMPLE_ESP_WIFI_PASS      "ab230304"
#define EXAMPLE_ESP_MAXIMUM_RETRY  50

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

//** define for uart setting
#define ECHO_TEST_TXD (GPIO_NUM_12)
#define ECHO_TEST_RXD (GPIO_NUM_13)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM (UART_NUM_2)
#define BUF_SIZE (1024)

//** define for ftp setting
#define FTP_SERVER "172.20.10.6"
#define FTP_USER "ChangChiaLun"
#define FTP_PASS "ab230304"

#define TAG "main"

extern CEspLcd *tft;

static lv_obj_t *lvCameraImage; // Camera image object

// 定义直线参数结构体
struct LinePara
{
    float k;
    float b;
};

// 获取直线参数
void getLinePara(float x1, float y1, float x2, float y2, LinePara &LP)
{
    double m = 0;

    // 计算分子
    m = x2 - x1;

    if (0 == m)
    {
        LP.k = 10000.0;
        LP.b = y1 - LP.k * x1;
    }
    else
    {
        LP.k = (y2 - y1) / (x2 - x1);
        LP.b = y1 - LP.k * x1;
    }
}

// 获取交点
bool getCross(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, Point2f &pt)
{

    LinePara para1, para2;
    getLinePara(x1, y1, x2, y2, para1);
    getLinePara(x3, y3, x4, y4, para2);

    // 判断是否平行
    if (abs(para1.k - para2.k) > 0.5)
    {
        pt.x = (para2.b - para1.b) / (para1.k - para2.k);
        pt.y = para1.k * pt.x + para1.b;

        return true;
    }
    else
    {
        return false;
    }
}
//** function to set wifi
/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();

    esp_netif_dhcpc_stop(my_sta);

    esp_netif_ip_info_t ip_info;

    IP4_ADDR(&ip_info.ip, 172, 20, 10, 3);
   	IP4_ADDR(&ip_info.gw, 172, 20, 10, 1);
   	IP4_ADDR(&ip_info.netmask, 255, 255, 255, 240);

    esp_netif_set_ip_info(my_sta, &ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            {.ssid = EXAMPLE_ESP_WIFI_SSID},
            {.password = EXAMPLE_ESP_WIFI_PASS}
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

void gui_boot_screen()
{
    static lv_style_t style;
    lv_style_init(&style);

    lv_style_set_radius(&style, LV_STATE_DEFAULT, 2);
    lv_style_set_bg_opa(&style, LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_style_set_bg_color(&style, LV_STATE_DEFAULT, LV_COLOR_MAKE(190, 190, 190));
    lv_style_set_border_width(&style, LV_STATE_DEFAULT, 2);
    lv_style_set_border_color(&style, LV_STATE_DEFAULT, LV_COLOR_MAKE(142, 142, 142));

    lv_style_set_pad_top(&style, LV_STATE_DEFAULT, 60);
    lv_style_set_pad_bottom(&style, LV_STATE_DEFAULT, 60);
    lv_style_set_pad_left(&style, LV_STATE_DEFAULT, 60);
    lv_style_set_pad_right(&style, LV_STATE_DEFAULT, 60);

    lv_style_set_text_color(&style, LV_STATE_DEFAULT, LV_COLOR_MAKE(102, 102, 102));
    lv_style_set_text_letter_space(&style, LV_STATE_DEFAULT, 5);
    lv_style_set_text_line_space(&style, LV_STATE_DEFAULT, 20);

    /*Create an object with the new style*/
    lv_obj_t *obj = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(obj, LV_LABEL_PART_MAIN, &style);
    lv_label_set_text(obj, "TTGO\n"
                           "demo!");
    lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 0, 0);
    wait_msec(3000);
}

// Function to calculate distance
float distance(int x1, int y1, int x2, int y2)
{
    // Calculating distance
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

void gui_init()
{
    // Create screen
    lv_obj_t *scr = lv_obj_create(nullptr, nullptr);
    lv_scr_load(scr);
}

esp_err_t updateCameraImage(const cv::Mat &img)
{
    // static variables because they must still be available when lv_task_handler() is called
    static Mat imgCopy;
    static lv_img_dsc_t my_img_dsc;

    if (img.empty())
    {
        ESP_LOGW(TAG, "Can't display empty image");
        return ESP_ERR_INVALID_ARG;
    }

    // convert image to bgr565 if needed
    if (img.type() == CV_8UC1)
    { // grayscale image
        cvtColor(img, imgCopy, COLOR_GRAY2BGR565, 1);
    }
    else if (img.type() == CV_8UC3)
    { // BGR888 image
        cvtColor(img, imgCopy, COLOR_BGR2BGR565, 1);
    }
    else if (img.type() == CV_8UC2)
    { // BGR565 image
        img.copyTo(imgCopy);
    }

    my_img_dsc.header.always_zero = 0;
    my_img_dsc.header.w = imgCopy.cols;
    my_img_dsc.header.h = imgCopy.rows;
    my_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    my_img_dsc.data_size = imgCopy.size().width * imgCopy.size().height;
    my_img_dsc.data = imgCopy.ptr<uchar>(0);

    lv_img_set_src(lvCameraImage, &my_img_dsc); /* Set the created file as image */
    lv_obj_set_pos(lvCameraImage, -40, 0);

    return ESP_OK;
}

// mode of the demo
enum class DisplayMode : uint8_t
{
    RGB,
    GRAYSCALE,
    BINARIZED,
    EDGES,
    NUM_OF_MODES
};
static DisplayMode currentDisplayMode;

static const std::string displayModeToString(DisplayMode dispMode)
{
    const std::map<DisplayMode, const std::string> DisplayModeStrings{
        {DisplayMode::RGB, "RGB"},
        {DisplayMode::GRAYSCALE, "GRAYSCALE"},
        {DisplayMode::BINARIZED, "BINARIZED"},
        {DisplayMode::EDGES, "EDGES"},
    };
    auto it = DisplayModeStrings.find(dispMode);
    return (it == DisplayModeStrings.end()) ? "Out of range" : it->second;
}

// static void tx_task()
// {
//     const char *Txdata = (char *)malloc(100);
//     Txdata = "AT+SEND=1,7,101,151";
//     int send_len = uart_tx_chars(ECHO_UART_PORT_NUM, Txdata, strlen(Txdata));
//     vTaskDelay(2000 / portTICK_PERIOD_MS);
//     cout << "send OK! " << send_len << endl;
//     // free(Txdata);
// }

/**
 * Task doing the demo: Getting image from camera, processing it with opencv depending on the displayMode and
 * displaying it on the lcd
 */

void demo_task(void *arg)
{
    static Mat line_image, src;
    ESP_LOGI(TAG, "Starting demo_task");

    // Display memory infos
    disp_infos();

    // tft->setRotation(2); // rotation needed if camera is on the back of the device
    sensor_t *s = esp_camera_sensor_get();

    // Init camera image Lvgl object
    // lvCameraImage = lv_img_create(lv_disp_get_scr_act(nullptr), nullptr);
    // lv_obj_move_foreground(lvCameraImage);
    gpio_pad_select_gpio(GPIO_NUM_4);                 // 选择一个GPIO
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT); // 把这个GPIO作为输出
    int out_count = 0;
    while (out_count < 50)
    {
        cout << out_count << endl;
        currentDisplayMode = DisplayMode::EDGES;
        gpio_set_level(GPIO_NUM_4, 1); // 把这个GPIO输出高電位
        // camera_fb_t *fb = esp_camera_fb_get();
        wait_msec(500);
        gpio_set_level(GPIO_NUM_4, 0); // 把这个GPIO输出低電位
        // auto start = esp_timer_get_time();
        // cout << "theta1=" << to_string(1) << ","
        //      << "theta2=" << to_string(51) << endl;
        // cout << "AT+SEND=1,"
        //      << "theta2=" << to_string(51) << endl;
        // int value1 = 1, value2 = 51;
        // int valuelen = to_string(value1 + 100).length() + to_string(value2 + 100).length() + 1;
        // String mymessage;
        // mymessage = mymessage + "AT+SEND=1" + "," + to_string(valuelen) + "," + to_string(value1 + 100) + "," + to_string(value2 + 100);
        // cout << mymessage << endl;
        // Setup UART buffered IO with event queue
        // Configure a temporary buffer for the incoming data
        uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 100);
        char chars[len + 1];
        memcpy(chars, data, len);
        char step = chars[9];
        //char step = '1';
        if (step == '1')
        {
            // send data by uart
            // int send_len = uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)test_str, strlen(test_str));
            // cout << "send " << send_len << endl;

            // int send_len = uart_write_bytes(UART_NUM_0, (const char *)test_str, strlen(test_str));
            // cout << "send " << send_len << endl;
            // tx_task();
            // wait_msec(500);
            gpio_set_level(GPIO_NUM_4, 1); // 把这个GPIO输出高電位
            camera_fb_t *fb = esp_camera_fb_get();
            gpio_set_level(GPIO_NUM_4, 0); // 把这个GPIO输出低電位
            int value1 = 0, value2 = 0;
            if (!fb)
            {
                ESP_LOGE(TAG, "Camera capture failed");
            }
            else
            {
                if (s->pixformat == PIXFORMAT_JPEG)
                {
                    // TFT_jpg_image(CENTER, CENTER, 0, -1, NULL, fb->buf, fb->len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                }
                else
                {                                                            // RGB565 pixformat
                    Mat inputImage(fb->height, fb->width, CV_8UC2, fb->buf); // rgb565 is 2 channels of 8-bit unsigned
                    cout << fb->height << endl;
                    cout << fb->width << endl;
                    inputImage = inputImage(Rect(560, 360, 300, 300));
                    if (currentDisplayMode == DisplayMode::RGB)
                    {
                    }
                    else if (currentDisplayMode == DisplayMode::GRAYSCALE)
                    {
                        cvtColor(inputImage, inputImage, COLOR_BGR5652GRAY);
                    }
                    else if (currentDisplayMode == DisplayMode::BINARIZED)
                    {
                        cvtColor(inputImage, inputImage, COLOR_BGR5652GRAY);
                        threshold(inputImage, inputImage, 128, 255, THRESH_BINARY);
                    }
                    else if (currentDisplayMode == DisplayMode::EDGES)
                    {
                        Mat inputImageCut = inputImage.clone();
                        cvtColor(inputImageCut, inputImageCut, COLOR_BGR5652GRAY);
                        cout << inputImageCut.rows << endl;
                        cout << inputImageCut.cols << endl;
                        // cout << "original pic" << inputImageCut << endl;
                        inputImageCut.copyTo(line_image);
                        inputImageCut.copyTo(src);
                        // cout << line_image.rows << endl;
                        // cout << line_image.cols << endl;
                        // cout << src.rows << endl;
                        // cout << src.cols << endl;
                        // Reduce noise with a kernel 3x3
                        GaussianBlur(inputImageCut, inputImageCut, Size(3, 3), 0);
                        // cout << "original pic" << inputImageCut << endl;
                        // Reduce line image noise with a kernel 3x3
                        // GaussianBlur(line_image, line_image, Size(3, 3), 0);
                        
                        /** Apply the canny edges detector with:
                         * - low threshold = 50
                         * - high threshold = 4x low
                         * - sobel kernel size = 3x3
                         */
                        // 霍夫圆检测
                        int lowThresh = 50;
                        int kernSize = 3;
                        Canny(inputImageCut, inputImageCut, lowThresh, 3 * lowThresh, kernSize);
                        // cv::resize(inputImage, resizeImg, cv::Size(40, 30), cv::INTER_LINEAR);
                        // cout << "original pic" << inputImage << endl;
                        // ESP_LOGE(TAG, "DetectCanny", inputImage);
                        vector<Vec3f> pcircles;
                        vector<Vec3f> min_circle;
                        HoughCircles(inputImageCut, pcircles, HOUGH_GRADIENT, 1, 30, 80, 30, 125, 140);
                        cout << pcircles.size() << endl;
                        if (pcircles.size() >= 1)
                        {
                            // 找最小圓且圓心在畫面中間
                            Vec3f cc;
                            float min_radis = 125;
                            size_t min_index = 0;
                            for (size_t i = 0; i < pcircles.size(); i++)
                            {
                                cc = pcircles[i];
                                if ((cc[2] <= min_radis) & (cc[0] >= 125) & (cc[0] <= 175) & (cc[1] >= 125) & (cc[1] <= 175))
                                {
                                    min_radis = cc[2];
                                    min_index = i;
                                }
                            }

                            // for (size_t i = 0; i < pcircles.size(); i++)
                            // {
                            cc = pcircles[min_index];
                            // cout << cc[0] << " " << cc[1] << " " << cc[2] << endl;
                            circle(src, Point(cc[0], cc[1]), cc[2], Scalar(0, 0, 255), 2, LINE_AA);
                            circle(src, Point(cc[0], cc[1]), 2, Scalar(125, 25, 255), 2, LINE_AA);
                            // }
                            // cout << "src=" << src << endl;
                            ESP_LOGE(TAG, "Detect circles =: %d", pcircles.size());
                            vector<Vec4i> lines;
                            // HoughLines(src1, lines, 1, CV_PI / 180, 150, 0, 0);
                            // 霍夫線检测
                            int lowThresh = 50;
                            int kernSize = 3;
                            // convertScaleAbs(line_image, line_image, 1.2, 0);
                            // line_image.copyTo(src);
                            Canny(line_image, line_image, lowThresh, 3 * lowThresh, kernSize);
                            // for (size_t start_x = cc[2] - 5; start_x <= cc[2] + 5; start_x++)
                            // {
                            //     for (size_t start_y = cc[2] - 5; start_y <= cc[2] + 5; start_y++)
                            //     {
                            //         if (line_image.at<cv::Vec3b>(i, j)[0] >= 230)
                            //         {
                            //             cout << start_x << " " << start_y << endl;
                            //         }
                            //     }
                            // }

                            HoughLinesP(line_image, lines, 1, CV_PI / 180, 1, 20, 1); // 进行霍夫线变换
                            // ESP_LOGE(TAG, "Detect lines: %d", lines.size());
                            ESP_LOGE(TAG, "Detect lines: %d", lines.size());
                            vector<Vec4i> linesInCircles;
                            Vec4i l, max_l;
                            Point2f line_pt;
                            // 確認線是否在圓內，圓外的排除
                            float maxline_dist = 0;
                            for (size_t i = 0; i < lines.size(); i++)
                            {
                                l = lines[i];
                                if ((distance(l[0], l[1], cc[0], cc[1]) <= cc[2] * 0.8) & (distance(l[2], l[3], cc[0], cc[1]) <= cc[2] * 0.8)) // 需在圓內
                                {
                                    if (distance(l[0], l[1], l[2], l[3]) > maxline_dist){
                                        maxline_dist = distance(l[0], l[1], l[2], l[3]);
                                        max_l = l;
                                    }
                                    linesInCircles.push_back(l);
                                    line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, LINE_AA);
                                }
                            }
                            // 由篩選後的線段進行角度判斷
                            if (linesInCircles.size() != 0)
                            {

                                if (linesInCircles.size() == 1)
                                {
                                    l = linesInCircles[0];
                                    if (distance(l[0], l[1], cc[0], cc[1]) < distance(l[2], l[3], cc[0], cc[1])){
                                        line_pt.x = l[2];
                                        line_pt.y = l[3];
                                    }else{
                                        line_pt.x = l[0];
                                        line_pt.y = l[1];
                                    }
                                    // line_pt.x = (l[0] + l[2] / 2);
                                    // line_pt.y = (l[1] + l[3] / 2);
                                }
                                else if (linesInCircles.size() >= 2)
                                {
                                    l = max_l;
                                    if (distance(l[0], l[1], cc[0], cc[1]) < distance(l[2], l[3], cc[0], cc[1])){
                                        line_pt.x = l[2];
                                        line_pt.y = l[3];
                                    }else{
                                        line_pt.x = l[0];
                                        line_pt.y = l[1];
                                    }
                                    // line_pt.x = (l[0] + l[2] / 2);
                                    // line_pt.y = (l[1] + l[3] / 2);
                                    // Vec4i l1 = linesInCircles[0];
                                    // Vec4i l2 = linesInCircles[1];
                                    // getCross(l1[0], l1[1], l1[2], l1[3], l2[0], l2[1], l2[2], l2[3], line_pt);
                                }
                                float true_theta = 0;
                                float delta_x = abs(line_pt.x - cc[0]);
                                float delta_y = abs(line_pt.y - cc[1]);
                                if (((line_pt.x - cc[0]) < 0) & ((l[1] - cc[1]) > 0))
                                {
                                    true_theta = CV_PI / 2 - atan(delta_y / delta_x);
                                }
                                else if (((line_pt.x - cc[0]) < 0) & ((line_pt.y - cc[1]) < 0))
                                {
                                    true_theta = CV_PI / 2 + atan(delta_y / delta_x);
                                }
                                else if (((line_pt.x - cc[0]) > 0) & ((line_pt.y - cc[1]) < 0))
                                {
                                    true_theta = 3 * CV_PI / 2 - atan(delta_y / delta_x);
                                }
                                else if (((line_pt.x - cc[0]) > 0) & ((line_pt.y - cc[1]) > 0))
                                {
                                    true_theta = 3 * CV_PI / 2 + atan(delta_y / delta_x);
                                }
                                // line(src, Point(line_pt.x, line_pt.y), Point(cc[0], cc[1]), Scalar(255, 0, 0), 1, LINE_AA);

                                // cout << "src=" << src << endl;
                                ESP_LOGE(TAG, "True theta: %.4f", true_theta / CV_PI * 180);
                                ESP_LOGE(TAG, "line length: %.4f", distance(line_pt.x, line_pt.y, cc[0], cc[1]));
                                int theta1 = (int)(true_theta / CV_PI * 180);
                                int theta2 = (theta1 + 180) % 360;
                                float range1 = 0.1818, range2 = 0.4285;

                                if (theta1 <= 110)
                                {
                                    value1 = (int)(theta1 * range1);
                                }
                                else if (theta1 < 250)
                                {
                                    value1 = (int)(20 + (theta1 - 110) * range2);
                                }
                                else if (theta1 >= 250)
                                {
                                    value1 = (int)(80 + (theta1 - 250) * range1);
                                }

                                if (theta2 <= 110)
                                {
                                    value2 = (int)(theta2 * range1);
                                }
                                else if (theta2 < 250)
                                {
                                    value2 = (int)(20 + (theta2 - 110) * range2);
                                }
                                else if (theta2 >= 250)
                                {
                                    value2 = (int)(80 + (theta2 - 250) * range1);
                                }
                            }
                            else
                            {
                                ESP_LOGE(TAG, "Not Detect Lines in Circles");
                            }
                            // for (size_t i = 0; i < lines.size(); i++)
                            // {
                            //     Vec4i l = lines[i];
                            //     // [x0, y0, x1, y1]
                            //     // line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1, LINE_AA);
                            //     if ((distance(l[0], l[1], cc[0], cc[1]) < cc[2]*0.5) & (distance(l[2], l[3], cc[0], cc[1]) < cc[2]*0.5)) // 需在圓內
                            //     {
                            //         if ((min(l[0], l[2]) - 5 <= cc[0]) & (cc[0] <= max(l[0], l[2]) + 5) & (min(l[1], l[3]) - 5 <= cc[1]) & (cc[1] <= max(l[1], l[3]) + 5)) // 線需大概包含圓心
                            //         {
                            //             float true_theta = 0;
                            //             float delta_x = abs(l[0] - cc[0]);
                            //             float delta_y = abs(l[1] - cc[1]);
                            //             if (((l[0] - cc[0]) < 0) & ((l[1] - cc[1]) > 0))
                            //             {
                            //                 true_theta = CV_PI / 2 - atan(delta_y / delta_x);
                            //             }
                            //             else if (((l[0] - cc[0]) < 0) & ((l[1] - cc[1]) < 0))
                            //             {
                            //                 true_theta = CV_PI / 2 + atan(delta_y / delta_x);
                            //             }
                            //             else if (((l[0] - cc[0]) > 0) & ((l[1] - cc[1]) < 0))
                            //             {
                            //                 true_theta = 3 * CV_PI / 2 - atan(delta_y / delta_x);
                            //             }
                            //             else if (((l[0] - cc[0]) > 0) & ((l[1] - cc[1]) > 0))
                            //             {
                            //                 true_theta = 3 * CV_PI / 2 + atan(delta_y / delta_x);
                            //             }
                            //             line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1, LINE_AA);

                            //             // cout << "src=" << src << endl;
                            //             ESP_LOGE(TAG, "True theta: %.4f", true_theta / CV_PI * 180);
                            //             ESP_LOGE(TAG, "line length: %.4f", distance(l[0], l[1], l[2], l[3]));
                            //             int theta1 = (int)(true_theta / CV_PI * 180);
                            //             int theta2 = (theta1 + 180) % 360;
                            //             float range1 = 0.1818, range2 = 0.4285;

                            //             if (theta1 <= 110)
                            //             {
                            //                 value1 = (int)(theta1 * range1);
                            //             }
                            //             else if (theta1 < 250)
                            //             {
                            //                 value1 = (int)(20 + (theta1 - 110) * range2);
                            //             }
                            //             else if (theta1 >= 250)
                            //             {
                            //                 value1 = (int)(80 + (theta1 - 250) * range1);
                            //             }

                            //             if (theta2 <= 110)
                            //             {
                            //                 value2 = (int)(theta2 * range1);
                            //             }
                            //             else if (theta2 < 250)
                            //             {
                            //                 value2 = (int)(20 + (theta2 - 110) * range2);
                            //             }
                            //             else if (theta2 >= 250)
                            //             {
                            //                 value2 = (int)(80 + (theta2 - 250) * range1);
                            //             }
                            //         }
                            //     }
                            // }
                            // cout << "src=" << src << endl;
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Detect Zeros Circle");
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Wrong display mode: %d", (int)currentDisplayMode);
                    }

                    // display image on lcd
                    // updateCameraImage(inputImage);
                }
            }
            int valuelen = to_string(value1 + 100).length() + to_string(value2 + 100).length() + 1;
            String mymessage;
            mymessage = mymessage + "AT+SEND=1" + "," + to_string(valuelen) + "," + to_string(value1 + 100) + "," + to_string(value2 + 100);
            int count = 0;
            while ((count < 5) & (step != '2'))
            {
                count += 1;
                cout << mymessage << endl;
                len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 100);
                memcpy(chars, data, len);
                step = chars[9];
                wait_msec(500);
            }
            uart_driver_delete(ECHO_UART_PORT_NUM);
            esp_sleep_enable_timer_wakeup(5000000);
            esp_deep_sleep_start();
            esp_restart();
            // ESP_LOGI(TAG, "%s mode: around %f fps", displayModeToString(currentDisplayMode).c_str(), 1.0f / ((esp_timer_get_time() - start) / 1000000.0f));
        }
        out_count += 1;
        wait_msec(500);
    }
    uart_driver_delete(ECHO_UART_PORT_NUM);
    esp_sleep_enable_timer_wakeup(10000000);
    esp_deep_sleep_start();
    esp_restart();
}

/**
 * Task changing the current displayMode at regular interval
 */
void timer_task(void *arg)
{
    while (true)
    {
        wait_msec(10000);
        currentDisplayMode = static_cast<DisplayMode>((static_cast<int>(currentDisplayMode) + 1) % static_cast<int>(DisplayMode::NUM_OF_MODES));
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Starting main");

    // //Initialize NVS
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //   ESP_ERROR_CHECK(nvs_flash_erase());
    //   ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    // wifi_init_sta();

    /* initializations */
    app_camera_init();
    // lvgl_init();
    // gui_init();

    /* display boot screen */
    // gui_boot_screen();

    /* Display memory infos */
    disp_infos();

    cout << "AT+NETWORKID=10" << endl;
    wait_msec(500);
    cout << "AT+ADDRESS=4" << endl;
    wait_msec(500);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    int intr_alloc_flags = 0;

    // First -> UART0
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Second -> UART2
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
    // ESP_LOGI(TAG, "Display width = %d, height = %d", tft->width(), tft->height());

    /* Define Lora ID & address */
    // const char *Txdata1 = (char *)malloc(100);
    // Txdata1 = "AT+NETWORKID=10";
    // uart_tx_chars(ECHO_UART_PORT_NUM, Txdata1, strlen(Txdata1));
    // const char *Txdata2 = (char *)malloc(100);
    // Txdata2 = "AT+ADDRESS=2";
    // uart_tx_chars(ECHO_UART_PORT_NUM, Txdata2, strlen(Txdata2));

    cout << "start test!";
    /* Start the tasks */
    xTaskCreatePinnedToCore(demo_task, "demo", 1024 * 9, nullptr, 24, nullptr, 0);
    // xTaskCreatePinnedToCore(timer_task, "timer", 1024 * 1, nullptr, 24, nullptr, 0);
}