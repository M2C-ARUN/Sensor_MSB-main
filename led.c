#include "float.h"
#include "led.h"

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
