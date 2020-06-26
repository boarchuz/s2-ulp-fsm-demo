#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp32s2/ulp.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"

static const char *TAG = "S2ULP";

#define ARRAY_COUNT(x) (sizeof(x) / sizeof((x)[0]))
#define RTC_WORD_OFFSET(x) ((uint16_t)((uint32_t*)&(x) - RTC_SLOW_MEM))

#define LED_GPIO GPIO_NUM_6

RTC_DATA_ATTR uint32_t ulp_run_count;
RTC_DATA_ATTR uint32_t ulp_upper_lower;
RTC_DATA_ATTR uint32_t ulp_arr[10];
RTC_DATA_ATTR uint32_t ulp_led_state;

void init_ulp()
{
    const ulp_insn_t program[] = {
        //Increment run counter
        I_MOVI(R3, RTC_WORD_OFFSET(ulp_run_count)),
        I_LD(R0, R3, 0),
        I_ADDI(R0, R0, 1),
        I_ST(R0, R3, 0),

        //Load, store upper and lower half-words
        I_LDU(R0, R3, 0),
        I_MOVI(R3, RTC_WORD_OFFSET(ulp_upper_lower)),
        I_ST16(R0, R3, 0, 0),
        I_MOVI(R0, 12345),
        I_ST16(R0, R3, 0, 1),

        //Demonstrate automatically incrementing store
        I_MOVI(R3, RTC_WORD_OFFSET(ulp_arr)),
        I_ST_OFFSET(0),
        I_MOVI(R0, 0),
        I_MOVI(R1, 100),
        //Loop to fill array
        I_ST_A(R1, R3),
        I_ADDI(R1, R1, 1),
        I_ADDI(R0, R0, 1),
        I_BRLT(-3, ARRAY_COUNT(ulp_arr)),

        //Toggle LED
        I_MOVI(R3, RTC_WORD_OFFSET(ulp_led_state)),
        I_LD(R0, R3, 0),
        I_BREQ(5, 0),
        I_MOVI(R0, 0),
        I_ST(R0, R3, 0),
        I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG, (uint8_t)(RTC_GPIO_OUT_DATA_W1TC_S + rtc_io_number_get(LED_GPIO)), 1),
        I_HALT(),
        I_MOVI(R0, 1),
        I_ST(R0, R3, 0),
        I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG, (uint8_t)(RTC_GPIO_OUT_DATA_W1TS_S + rtc_io_number_get(LED_GPIO)), 1),
        I_HALT(),
    };

    ESP_ERROR_CHECK(rtc_gpio_init(LED_GPIO));
    ESP_ERROR_CHECK(gpio_set_pull_mode(LED_GPIO, GPIO_FLOATING));
    ESP_ERROR_CHECK(rtc_gpio_set_level(LED_GPIO, 0));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(LED_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY));

    size_t program_size = ARRAY_COUNT(program);
    ESP_ERROR_CHECK(ulp_process_macros_and_load(0, program, &program_size));
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 1000 * 1000));
    ESP_ERROR_CHECK(ulp_run(0));
}

void app_main(void)
{
    init_ulp();
    
    //Wait for ULP to run through one time
    while( !(REG_READ(RTC_CNTL_LOW_POWER_ST_REG) & RTC_CNTL_COCPU_STATE_DONE) );

    ESP_LOGI(TAG, "auto incremented offset array:");
    for(int i = 0; i < ARRAY_COUNT(ulp_arr); ++i)
    {
        ESP_LOGI(TAG, "\t[%d]: %u", i, ulp_arr[i] & 0xFFFF);
    }
    ESP_LOGI(TAG, "ulp_upper_lower: %u (%u | %u)", ulp_upper_lower, (ulp_upper_lower >> 16) & 0xFFFF, (ulp_upper_lower >> 0) & 0xFFFF);

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    for(;;)
    {
        ESP_LOGI(TAG, "ulp_run_count: %u, ulp_led_state: %u", ulp_run_count & 0xFFFF, ulp_led_state & 0xFFFF);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
