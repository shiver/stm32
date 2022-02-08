#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <stm32f411xx.h>

void delay() {
  for (uint32_t volatile i = 0; i < 500000; i++);
}

/*
 * Toggles an LED on and off in open-drain mode without using the internal pull-up resistor.
 * Instead we use an external pull-up to get the job done.
 *
 * Setup:
 * - Connect PB1 to VDD (5V) via a small resistor (such as 330ohm)
 * - Connect PB1 to the LED
 * - Connect the LED to another smallish resistor to avoid overloading the LED
 * - Connect the led-resistor to GND
 */
void ledWithOpenDrainAndNoInternalPullUp() {
  GPIO_Handle_t led2 = {
        .port = GPIOB,
        .pinConfig = {
            .number = 1,
            .mode = GPIO_MODE_OUTPUT,
            .pullUpPullDown = GPIO_PUPD_NONE,
            .outputType = GPIO_OUTPUT_TYPE_OPEN_DRAIN,
        }
    };

    gpio_init(&led2);

    for(;;) {
      gpio_writePin(&led2, 1);
      delay();
      gpio_writePin(&led2, 0);
      delay();
    }
}

/*
 * Toggles an LED on and off in open-drain mode using the internal pull-up resistor.
 *
 * Setup:
 * - Connect PB1 to the LED
 * - Connect the LED to a smallish resistor to avoid overloading the LED
 * - Connect the resistor to GND
 */
void ledWithOpenDrainAndInternalPullUp() {
  GPIO_Handle_t led2 = {
        .port = GPIOB,
        .pinConfig = {
            .number = 1,
            .mode = GPIO_MODE_OUTPUT,
            .pullUpPullDown = GPIO_PUPD_PULL_UP,
            .outputType = GPIO_OUTPUT_TYPE_OPEN_DRAIN,
        }
    };

    gpio_init(&led2);

    for(;;) {
      gpio_writePin(&led2, 1);
      delay();
      gpio_writePin(&led2, 0);
      delay();
    }
}

/*
 * Toggles an LED on and off in push-pull mode and no internal or external pull-up resistors.
 *
 * Setup:
 * - Connect PB1 to the LED
 * - Connect the LED to a smallish resistor to avoid overloading the LED
 * - Connect the resistor to GND
 */
void ledWithPushPull() {
  GPIO_Handle_t led2 = {
        .port = GPIOB,
        .pinConfig = {
            .number = 1,
            .mode = GPIO_MODE_OUTPUT,
            .pullUpPullDown = GPIO_PUPD_NONE,
            .outputType = GPIO_OUTPUT_TYPE_PUSH_PULL,
        }
    };

    gpio_init(&led2);

    for(;;) {
      gpio_writePin(&led2, 1);
      delay();
      gpio_writePin(&led2, 0);
      delay();
    }
}

int main(void)
{
  ledWithPushPull();
}
