/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "calendar.h"
#include "nrf.h"
#include "main.h"
#include "flash_func.h"

extern configuration_t m_device_cfg;

static struct tm time_struct, m_tm_return_time;
static time_t m_time, m_last_calibrate_time = 0;
static float m_calibrate_factor = 0.0f;
static uint32_t m_rtc_increment = 60;
static void (*cal_event_callback)(void) = 0;

APP_TIMER_DEF(TIMER_CAL);
/**
 * @brief RTC interrupt handler.
 */
void timer_test_handler(void *p_context)
{
    static int count = 0;
    NRF_LOG_INFO("tIMER %d", count++);
}
/**
 * @brief Function for initializing the Calendar module.
 */
void nrf_cal_init(void)
{
    // Select the 32 kHz crystal and start the 32 kHz clock
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
        ;

    // Configure the RTC for 1 minute wakeup (default)
    CAL_RTC->PRESCALER = 0xFFF;
    CAL_RTC->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    CAL_RTC->INTENSET = RTC_INTENSET_COMPARE0_Msk;
    CAL_RTC->CC[0] = m_rtc_increment * 8;
    CAL_RTC->TASKS_START = 1;
    NVIC_SetPriority(CAL_RTC_IRQn, CAL_RTC_IRQ_Priority);
    NVIC_EnableIRQ(CAL_RTC_IRQn);
}
/**
 * @brief Function for setting the calendar callback.
 */
void nrf_cal_set_callback(void (*callback)(void), uint32_t interval)
{
    // Set the calendar callback, and set the callback interval in seconds
    cal_event_callback = callback;
    m_rtc_increment = interval;
    m_time += CAL_RTC->COUNTER / 8;
    CAL_RTC->TASKS_CLEAR = 1;
    CAL_RTC->CC[0] = interval * 8;
}
/**
 * @brief Function for setting the calendar time.
 */
void nrf_cal_set_time(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second)
{
    int data = 0;
    static time_t uncal_difftime, difftime, newtime;
    time_struct.tm_year = year - 1900;
    time_struct.tm_mon = month;
    time_struct.tm_mday = day;
    time_struct.tm_hour = hour;
    time_struct.tm_min = minute;
    time_struct.tm_sec = second;
    newtime = mktime(&time_struct);
    CAL_RTC->TASKS_CLEAR = 1;
    // Calculate the calibration offset
    if (m_last_calibrate_time != 0)
    {
        difftime = newtime - m_last_calibrate_time;
        uncal_difftime = m_time - m_last_calibrate_time;
        m_calibrate_factor = (float)difftime / (float)uncal_difftime;
    }

    // Assign the new time to the local time variables
    m_time = m_last_calibrate_time = newtime - 19800;

    m_device_cfg.rtc_time = m_time;
    setTime();
}
/**
 * @brief Function for setting the calendar time using time_t.
 */
void nrf_cal_set_time_t(time_t time)
{
    static time_t uncal_difftime, difftime, newtime;
    newtime = time;

    // Calculate the calibration offset
    if (m_last_calibrate_time != 0)
    {
        difftime = newtime - m_last_calibrate_time;
        uncal_difftime = m_time - m_last_calibrate_time;
        m_calibrate_factor = (float)difftime / (float)uncal_difftime;
    }

    // Assign the new time to the local time variables
    m_time = m_last_calibrate_time = newtime;
}
/**
 * @brief Function for getting the calendar time.
 */
struct tm *nrf_cal_get_time(void)
{
    time_t return_time;
    return_time = m_time + CAL_RTC->COUNTER / 8;
    m_tm_return_time = *localtime(&return_time);
    return &m_tm_return_time;
}
/**
 * @brief Function for getting the calibrated calendar time.
 */
struct tm *nrf_cal_get_time_calibrated(void)
{
    time_t uncalibrated_time, calibrated_time;
    if (m_calibrate_factor != 0.0f)
    {
        uncalibrated_time = m_time + CAL_RTC->COUNTER / 8;
        calibrated_time = m_last_calibrate_time + (time_t)((float)(uncalibrated_time - m_last_calibrate_time) * m_calibrate_factor + 0.5f);
        m_tm_return_time = *localtime(&calibrated_time);
        return &m_tm_return_time;
    }
    else
        return nrf_cal_get_time();
}
/**
 * @brief Function for getting the calendar time as a string.
 */
char *nrf_cal_get_time_string(bool calibrated)
{
    static char cal_string[80];
    struct tm *timeInfo = calibrated ? nrf_cal_get_time_calibrated() : nrf_cal_get_time();

    // Adjust for the time zone offset
    timeInfo->tm_hour += 5;

    strftime(cal_string, 80, "%x   %H:%M:%S", timeInfo);
    // strftime(cal_string, 80, "%x   %H:%M:%S", (calibrated ? nrf_cal_get_time_calibrated() : nrf_cal_get_time()));
    return cal_string;
}
/**
 * @brief Function for getting the epoch time.
 */
char *nrf_cal_get_epoch_time_string(time_t *tt)
{

    // in this function we get time & Date : day/month/year & hour/minutes/senconds

    struct tm t, m_tm_return_time1;
    time_t return_time;
    return_time = m_time + CAL_RTC->COUNTER / 8;
    m_tm_return_time1 = *localtime(&return_time);
    t.tm_year = m_tm_return_time1.tm_year;
    t.tm_mon = m_tm_return_time1.tm_mon;
    ;
    t.tm_mday = m_tm_return_time1.tm_mday;
    t.tm_hour = m_tm_return_time1.tm_hour;
    t.tm_min = m_tm_return_time1.tm_min;
    t.tm_sec = m_tm_return_time1.tm_sec;
    *tt = mktime(&t);
}
/**
 * @brief RTC interrupt handler.
 */
void CAL_RTC_IRQHandler(void)
{
    if (CAL_RTC->EVENTS_COMPARE[0])
    {
        CAL_RTC->EVENTS_COMPARE[0] = 0;

        CAL_RTC->TASKS_CLEAR = 1;

        m_time += m_rtc_increment;
        m_device_cfg.rtc_time = m_time;
        if (cal_event_callback)
            cal_event_callback();
    }
}