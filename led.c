#include "float.h"
#include "led.h"

// Example of LED brightness control using PWM
// void led_brightness_control(void)
// {
// Initialize PWM
// pwm_init();

// // Fade LED up and down
// while (1)
// {
// Fade up
//     for (uint16_t i = 0; i <= 100; i++)
//     {
//         pwm_set_duty_cycle(i);
//         // nrf_delay_ms(50); // Adjust speed of fading
//         printf("increse\n");
//     }

//     // Fade down
//     for (uint16_t i = 100; i > 0; i--)
//     {
//         pwm_set_duty_cycle(i);
//         // nrf_delay_ms(50);
//         printf("decrese\n");
//     }
//     // }
// }
void fade_led() 
{
    static bool increasing = true;
    static uint16_t brightness = 0;

    if (increasing)
    {
        brightness++;
        if (brightness >= 100)
        {
            increasing = false;
        }
    }
    else
    {
        brightness--;
        if (brightness == 0)
        {
            increasing = true;
        }
    }

    pwm_set_duty_cycle(brightness);
}
