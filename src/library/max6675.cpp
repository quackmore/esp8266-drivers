/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <quackmore-ff@yahoo.com> modified this file.  As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy me a beer in return. Quackmore
 * ----------------------------------------------------------------------------
 */

extern "C"
{
#include "c_types.h"
#include "osapi.h"
#include "gpio.h"
#include "mem.h"
#include "user_interface.h"
#include "esp8266_io.h"
#include "library_do_sequence.h"
#include "library_di_sequence.h"

#ifdef ESPBOT
    // these are espbot_2.0 memory management methods
    // https://github.com/quackmore/espbot_2.0
    extern void *call_espbot_zalloc(size_t size);
    extern void call_espbot_free(void *addr);
#else
#define call_espbot_zalloc(a) os_zalloc(a)
#define call_espbot_free(a) os_free(a)
#define getTimestamp() system_get_time();
#endif
}

#include "library_max6675.hpp"

#ifdef ESPBOT

#include "espbot_global.hpp"
#define PRINT_FATAL(...) esplog.fatal(__VA_ARGS__)
#define PRINT_ERROR(...) esplog.error(__VA_ARGS__)
#define PRINT_WARN(...) esplog.warn(__VA_ARGS__)
#define PRINT_INFO(...) esplog.info(__VA_ARGS__)
#define PRINT_DEBUG(...) esplog.debug(__VA_ARGS__)
#define PRINT_TRACE(...) esplog.trace(__VA_ARGS__)
#define PRINT_ALL(...) esplog.all(__VA_ARGS__)
#define getTimestamp() esp_sntp.get_timestamp();

#else

#define PRINT_FATAL(...) os_printf(__VA_ARGS__)
#define PRINT_ERROR(...) os_printf(__VA_ARGS__)
#define PRINT_WARN(...) os_printf(__VA_ARGS__)
#define PRINT_INFO(...) os_printf(__VA_ARGS__)
#define PRINT_DEBUG(...) os_printf(__VA_ARGS__)
#define PRINT_TRACE(...) os_printf(__VA_ARGS__)
#define PRINT_ALL(...) os_printf(__VA_ARGS__)

#endif

static void ICACHE_FLASH_ATTR max6675_read_completed(Max6675 *max6675_ptr)
{
    int cur_pos;
    if (max6675_ptr->m_buffer_idx == (max6675_ptr->m_max_buffer_size - 1))
        cur_pos = 0;
    else
        cur_pos = max6675_ptr->m_buffer_idx + 1;
    // check if the reading is valid
    // thermocouple disconnected => bit 2 is high
    if (max6675_ptr->m_data & 0x0004)
    {
        PRINT_ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] thermocouple disconnected\n",
                    max6675_ptr->m_cs,
                    max6675_ptr->m_sck,
                    max6675_ptr->m_so);
        return;
    }
    // set the value as valid
    max6675_ptr->m_invalid_buffer[cur_pos] = false;
    // set the timestamp
    max6675_ptr->m_timestamp_buffer[cur_pos] = getTimestamp();
    // set the value bits: 12 bits from 3 to 14
    max6675_ptr->m_temperature_buffer[cur_pos] = (max6675_ptr->m_data >> 3) & 0x0FFF;
    // update the buffer position
    if (max6675_ptr->m_buffer_idx == (max6675_ptr->m_max_buffer_size - 1))
        max6675_ptr->m_buffer_idx = 0;
    else
        max6675_ptr->m_buffer_idx++;
}

static void ICACHE_FLASH_ATTR max6675_read_bit(void *param)
{
    Max6675 *max6675_ptr = (Max6675 *)param;

    if (max6675_ptr->m_bit_counter < 32)
    {
        // CS  _                                 _
        //      |_____________________ .._______|
        // SCK   _   _   _   _   _   _ ..  _   _
        //     _| |_| |_| |_| |_| |_| |.._| |_| |_
        //
        // current bit reading
        if (((max6675_ptr->m_bit_counter) % 2) == 0)
        {
            // on even counter set SCK high and read SO
            GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->m_sck), ESPBOT_HIGH);
            int bit_value = GPIO_INPUT_GET(gpio_NUM(max6675_ptr->m_so));
            char bit_idx = max6675_ptr->m_bit_counter / 2;
            if (bit_value)
                max6675_ptr->m_data |= (0x01 << (15 - bit_idx));
        }
        else
        {
            // on odd counter set SCK low
            GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->m_sck), ESPBOT_LOW);
        }
        // setup for next bit reading
        max6675_ptr->m_bit_counter++;
        os_timer_disarm(&(max6675_ptr->m_read_timer));
        os_timer_setfn(&(max6675_ptr->m_read_timer), max6675_read_bit, (void *)max6675_ptr);
        os_timer_arm(&(max6675_ptr->m_read_timer), 5, 0);
    }
    else
    {
        // reading completed
        // set SCK low
        GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->m_sck), ESPBOT_LOW);
        // set CS high
        GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->m_cs), ESPBOT_HIGH);
        max6675_read_completed(max6675_ptr);
    }
}

static void ICACHE_FLASH_ATTR max6675_read(Max6675 *max6675_ptr)
{
    // configure CS as output and set it LOW
    PIN_FUNC_SELECT(gpio_MUX(max6675_ptr->m_cs), gpio_FUNC(max6675_ptr->m_cs));
    GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->m_cs), ESPBOT_LOW);
    // configure SCK as output and set it LOW
    PIN_FUNC_SELECT(gpio_MUX(max6675_ptr->m_sck), gpio_FUNC(max6675_ptr->m_sck));
    GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->m_sck), ESPBOT_LOW);
    // configure SO as input
    PIN_FUNC_SELECT(gpio_MUX(max6675_ptr->m_so), gpio_FUNC(max6675_ptr->m_so));
    GPIO_DIS_OUTPUT(gpio_NUM(max6675_ptr->m_so));
    // clear current readings
    max6675_ptr->m_data = 0;
    max6675_ptr->m_bit_counter = 0;
    // start the SCK sequence and SO reading
    os_timer_disarm(&(max6675_ptr->m_read_timer));
    os_timer_setfn(&(max6675_ptr->m_read_timer), max6675_read_bit, (void *)max6675_ptr);
    os_timer_arm(&(max6675_ptr->m_read_timer), 5, 0);
}

ICACHE_FLASH_ATTR Max6675::Max6675()
{
}

ICACHE_FLASH_ATTR Max6675::~Max6675()
{
    call_espbot_free(m_temperature_buffer);
    call_espbot_free(m_timestamp_buffer);
}

void ICACHE_FLASH_ATTR Max6675::init(int cs_pin, int sck_pin, int so_pin, int poll_interval, int buffer_length)
{
    // init variables
    int idx;
    m_cs = cs_pin;
    m_sck = sck_pin;
    m_so = so_pin;
    m_poll_interval = poll_interval;
    if (m_poll_interval < 1)
        m_poll_interval = 1;

    m_temperature_buffer = (int *)call_espbot_zalloc(buffer_length * sizeof(int));
    for (idx = 0; idx < buffer_length; idx++)
        m_temperature_buffer[idx] = 0;
    m_timestamp_buffer = (uint32_t *)call_espbot_zalloc(buffer_length * sizeof(int));
    for (idx = 0; idx < buffer_length; idx++)
        m_timestamp_buffer[idx] = 0;
    m_invalid_buffer = (bool *)call_espbot_zalloc(buffer_length * sizeof(bool));
    for (idx = 0; idx < buffer_length; idx++)
        m_invalid_buffer[idx] = true;

    m_max_buffer_size = buffer_length;
    m_buffer_idx = 0;
    m_retry = false;

    // set CS high
    PIN_FUNC_SELECT(gpio_MUX(m_cs), gpio_FUNC(m_cs));
    GPIO_OUTPUT_SET(gpio_NUM(m_cs), ESPBOT_HIGH);
    // start polling
    os_timer_disarm(&m_poll_timer);
    os_timer_setfn(&m_poll_timer, (os_timer_func_t *)max6675_read, this);
    os_timer_arm(&m_poll_timer, m_poll_interval * 1000, 1);
}

float ICACHE_FLASH_ATTR Max6675::get_temperature(Temp_scale scale, int idx)
{
    int index = m_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = m_max_buffer_size - 1;
        idx--;
    }

    if (scale == Celsius)
        return ((float)m_temperature_buffer[index] / 4);
    else if (scale == Fahrenheit)
        return ((((float)m_temperature_buffer[index] / 4) * 1.8) + 32);
}

uint32_t ICACHE_FLASH_ATTR Max6675::get_timestamp(int idx)
{
    int index = m_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = m_max_buffer_size - 1;
        idx--;
    }
    return m_timestamp_buffer[index];
}

bool ICACHE_FLASH_ATTR Max6675::get_invalid(int idx)
{
    int index = m_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = m_max_buffer_size - 1;
        idx--;
    }
    return m_invalid_buffer[index];
}