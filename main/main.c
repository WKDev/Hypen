#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"

#include "sdkconfig.h"

// GPIO0, GPIO2, GPIO5, GPIO12 (MTDI), and GPIO15 (MTDO) are strapping pin
// use rtc GPIO to wake up with mpu
// 풀업 활성화시 gnd와 연결 하면 활성화됨, 기본값이 0임
#define SELECT_BTN 15
#define LED1 26
#define LED2 27
#define LED3 12
#define NAV_U 32
#define NAV_R 33
#define NAV_D 25
#define NAV_L 16
#define VMONITOR 35
#define TP_CHRG 14
#define EN_VCHECK 13

#define MPU_INT 23
#define MPU_SCL 22
#define MPU_SDA 21
#define NAV_C 19
#define NOTI_LED1 18
#define NOTI_LED2 5
#define NOTI_LED3 17
#define TOUCH 4
#define LED_BUILTIN 2

static const char *TAG = "TOUCHPAD";

#define TOUCH_THRESH_NO_USE (0)
#define TOUCH_THRESH_PERCENT (80)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

static bool s_pad_activated;
static uint32_t s_pad_init_val;

#define INPUT_BTN_SELECTION (1ULL << SELECT_BTN) | (1ULL << NAV_U) | (1ULL << NAV_D) | (1ULL << NAV_L) | (1ULL << NAV_R) | (1ULL << NAV_C)
#define OUTPUT_LED_SELECTION (1ULL << LED1) | (1ULL << LED3) | (1ULL << LED2) | (1ULL << 2)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle btn_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR btn_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(btn_evt_queue, &gpio_num, NULL);
}

static void nav_btn_action(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void btn_select(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(btn_evt_queue, &io_num, portMAX_DELAY))
        {
            printf("Change or pair new Device!! %d\n", gpio_get_level(io_num));
        }
    }
}

void gpio_initialization(gpio_config_t io_conf)
{
    /// 선택 버튼 GPIO 매핑
    //적용할 gpio
    io_conf.pin_bit_mask = INPUT_BTN_SELECTION;
    // rising edge에 인터럽트
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // 핀 모드 설정
    io_conf.mode = GPIO_MODE_INPUT;
    //풀-업 활성화
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    //io_conf 값으로 저장
    gpio_config(&io_conf);

    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // io_conf.pin_bit_mask = OUTPUT_LED_SELECTION;
    // // io_conf.pull_down_en = 1;
    // gpio_config(&io_conf);
}

void led_task(void *pvParameter)
{
    gpio_pad_select_gpio(LED1);
    gpio_pad_select_gpio(LED2);
    gpio_pad_select_gpio(LED3);
    gpio_pad_select_gpio(LED_BUILTIN);

    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);

    for (;;)
    {
        for (int i = 0; i < 5; i++)
        {
            gpio_set_level(LED3, 0);
            gpio_set_level(LED1, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED1, 0);
            gpio_set_level(LED2, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED2, 0);
            gpio_set_level(LED3, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        gpio_set_level(LED_BUILTIN, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(LED_BUILTIN, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(LED_BUILTIN, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(LED_BUILTIN, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static void tp_example_read_task(void *pvParameter)
{
    static int show_message;
    int change_mode = 0;
    int filter_mode = 0;
    while (1)
    {
        if (filter_mode == 0)
        {
            //interrupt mode, enable touch interrupt
            touch_pad_intr_enable();
            if (s_pad_activated == true)
            {
                ESP_LOGI(TAG, "T activated!");
                // Wait a while for the pad being released
                vTaskDelay(200 / portTICK_PERIOD_MS);
                // Clear information on pad activation
                s_pad_activated = false;
                // Reset the counter triggering a message
                // that application is running
                show_message = 1;
            }
        }

        else
        {
            //filter mode, disable touch interrupt
            touch_pad_intr_disable();
            touch_pad_clear_status();

            uint16_t value = 0;
            touch_pad_read_filtered(0, &value);
            if (value < s_pad_init_val * TOUCH_THRESH_PERCENT / 100)
            {
                ESP_LOGI(TAG, "Tactivated!");
                ESP_LOGI(TAG, "value: %d; init val: %d", value, s_pad_init_val);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                // Reset the counter to stop changing mode.
                change_mode = 1;
                show_message = 1;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);

        // If no pad is touched, every couple of seconds, show a message
        // that application is running
        if (show_message++ % 500 == 0)
        {
            ESP_LOGI(TAG, "Waiting for any pad being touched...");
        }
        // Change mode if no pad is touched for a long time.
        // We can compare the two different mode.
        if (change_mode++ % 2000 == 0)
        {
            filter_mode = !filter_mode;
            ESP_LOGW(TAG, "Change mode...%s", filter_mode == 0 ? "interrupt mode" : "filter mode");
        }
    }
}

static void tp_example_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    {
        if ((pad_intr >> 0) & 0x01)
        {
            s_pad_activated = true;
        }
    }
}

void app_main(void)
{

    //---------------------------Physical IO Section--------------------
    ////button

    //GPIO의 특성을 설정하는 io_conf 객체 생성
    gpio_config_t io_conf;

    gpio_initialization(io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(SELECT_BTN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(NAV_U, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(NAV_D, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(NAV_L, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(NAV_R, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(NAV_C, GPIO_INTR_ANYEDGE);

    //isr로부터 gpio event를 다루기 위한 큐 생성

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    btn_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(nav_btn_action, "nav_btn_action", 2048, NULL, 10, NULL);
    xTaskCreate(btn_select, "btn_select", 2048, NULL, 10, NULL);
    xTaskCreate(&led_task, "LEDTask", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    //gpio isr 서비스 설치
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //isr 핸들러를 특성 핀과 연결
    gpio_isr_handler_add(SELECT_BTN, btn_isr_handler, (void *)SELECT_BTN);
    gpio_isr_handler_add(NAV_U, gpio_isr_handler, (void *)NAV_U);
    gpio_isr_handler_add(NAV_D, gpio_isr_handler, (void *)NAV_D);
    gpio_isr_handler_add(NAV_L, gpio_isr_handler, (void *)NAV_L);
    gpio_isr_handler_add(NAV_R, gpio_isr_handler, (void *)NAV_R);
    gpio_isr_handler_add(NAV_C, gpio_isr_handler, (void *)NAV_C);

    ////Touch
    ESP_LOGI(TAG, "Initializing touch pad");
    ESP_ERROR_CHECK(touch_pad_init());

    // If use interrupt trigger mode, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // For most usage scenarios, we recommend using the following combination:
    // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference voltage will be 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    touch_pad_config(0, TOUCH_THRESH_NO_USE);
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);

    touch_pad_read_filtered(0, &s_pad_init_val);
    ESP_LOGI(TAG, "test init: touch pad val is %d", s_pad_init_val);
    //set interrupt threshold.
    ESP_ERROR_CHECK(touch_pad_set_thresh(0, s_pad_init_val * 2 / 3));

    // Register touch interrupt ISR
    touch_pad_isr_register(tp_example_rtc_intr, NULL);
    // Start a task to show what pads have been touched
    xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 2048, NULL, 1, NULL);
}
