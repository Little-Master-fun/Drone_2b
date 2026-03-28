#include "FreeRTOS.h"
#include "task.h"
#include "tasks_blink.h"
#include "zf_common_headfile.h"

#define TASKS_BLINK_LED_PIN            (P08_1)
#define TASKS_BLINK_STACK_WORDS        (128U)
#define TASKS_BLINK_PRIORITY           (1U)
#define TASKS_BLINK_PERIOD_TICKS       pdMS_TO_TICKS(500U)

static TaskHandle_t g_tasks_blink_handle = NULL;

static void tasks_blink_entry (void *parameter)
{
    (void)parameter;

    while(1)
    {
        gpio_toggle_level(TASKS_BLINK_LED_PIN);
        vTaskDelay(TASKS_BLINK_PERIOD_TICKS);
    }
}

void tasks_blink_init (void)
{
    gpio_init(TASKS_BLINK_LED_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    (void)xTaskCreate(tasks_blink_entry,
                      "blink",
                      TASKS_BLINK_STACK_WORDS,
                      NULL,
                      TASKS_BLINK_PRIORITY,
                      &g_tasks_blink_handle);
}
