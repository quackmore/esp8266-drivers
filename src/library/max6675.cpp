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
#include "esp8266_io.h"
}

#include "library.hpp"
#include "library_max6675.hpp"

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
        // done with reading
        max6675_ptr->m_reading_ongoing = false;
        // still something to do if it was a force reading
        if (max6675_ptr->m_force_reading)
        {
            max6675_ptr->m_force_reading = false;
            // restart polling
            os_timer_disarm(&(max6675_ptr->m_poll_timer));
            if (max6675_ptr->m_poll_interval > 0)
                os_timer_arm(&(max6675_ptr->m_poll_timer), max6675_ptr->m_poll_interval, 1);
            // actually there was no reading but anyway ...
            max6675_ptr->m_force_reading_cb(max6675_ptr->m_force_reading_param);
        }
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
    // done with reading
    max6675_ptr->m_reading_ongoing = false;
    // still something to do if it was a force reading
    if (max6675_ptr->m_force_reading)
    {
        max6675_ptr->m_force_reading = false;
        // restart polling
        os_timer_disarm(&(max6675_ptr->m_poll_timer));
        if (max6675_ptr->m_poll_interval > 0)
            os_timer_arm(&(max6675_ptr->m_poll_timer), max6675_ptr->m_poll_interval, 1);

        max6675_ptr->m_force_reading_cb(max6675_ptr->m_force_reading_param);
    }
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
    max6675_ptr->m_reading_ongoing = true;
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

ICACHE_FLASH_ATTR Max6675::Max6675(int cs_pin,
                                   int sck_pin,
                                   int so_pin,
                                   int id,
                                   int poll_interval,
                                   int buffer_length)
{
    // init variables
    int idx;
    m_cs = cs_pin;
    m_sck = sck_pin;
    m_so = so_pin;
    m_id = id;

    m_poll_interval = poll_interval;
    m_max_buffer_size = buffer_length;

    m_temperature_buffer = new int[m_max_buffer_size];
    if (m_temperature_buffer == NULL)
    {
        PRINT_ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] not enough heap memory\n");
        return;
    }
    for (idx = 0; idx < m_max_buffer_size; idx++)
        m_temperature_buffer[idx] = 0;
    m_timestamp_buffer = new uint32_t[m_max_buffer_size];
    if (m_timestamp_buffer == NULL)
    {
        PRINT_ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] not enough heap memory\n");
        return;
    }
    for (idx = 0; idx < m_max_buffer_size; idx++)
        m_timestamp_buffer[idx] = 0;
    m_invalid_buffer = new bool[m_max_buffer_size];
    if (m_invalid_buffer == NULL)
    {
        PRINT_ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] not enough heap memory\n");
        return;
    }
    for (idx = 0; idx < m_max_buffer_size; idx++)
        m_invalid_buffer[idx] = true;
    m_buffer_idx = 0;

    m_force_reading = false;
    m_reading_ongoing = false;

    // set CS high
    PIN_FUNC_SELECT(gpio_MUX(m_cs), gpio_FUNC(m_cs));
    GPIO_OUTPUT_SET(gpio_NUM(m_cs), ESPBOT_HIGH);
    // start polling
    os_timer_disarm(&m_poll_timer);
    os_timer_setfn(&m_poll_timer, (os_timer_func_t *)max6675_read, this);
    if (m_poll_interval > 0)
        os_timer_arm(&m_poll_timer, m_poll_interval, 1);
}

ICACHE_FLASH_ATTR Max6675::~Max6675()
{
    if (m_temperature_buffer)
        delete[] m_temperature_buffer;
    if (m_timestamp_buffer)
        delete[] m_timestamp_buffer;
    if (m_invalid_buffer)
        delete[] m_invalid_buffer;
}

int ICACHE_FLASH_ATTR Max6675::get_max_events_count(void)
{
    return m_max_buffer_size;
}

void ICACHE_FLASH_ATTR Max6675::force_reading(void (*callback)(void *), void *param)
{
    // if the class was not properly allocated exit
    if ((m_timestamp_buffer == NULL) ||
        (m_invalid_buffer == NULL) ||
        (m_temperature_buffer == NULL))
        return;
    m_force_reading_cb = callback;
    m_force_reading_param = param;
    m_force_reading = true;
    // in case a reading is ongoing do nothing
    // else stop the polling timer and force a reading start
    if (!m_reading_ongoing)
    {
        // stop_polling
        os_timer_disarm(&m_poll_timer);
        max6675_read(this);
    }
}

void ICACHE_FLASH_ATTR Max6675::getEvent(sensors_event_t *event, int idx)
{
    os_memset(event, 0, sizeof(sensors_event_t));
    event->sensor_id = m_id;
    event->type = SENSOR_TYPE_TEMPERATURE;
    // find the idx element
    int index = m_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = m_max_buffer_size - 1;
        idx--;
    }
    // if the class was not properly allocated exit
    if ((m_timestamp_buffer == NULL) || (m_invalid_buffer == NULL) || (m_temperature_buffer == NULL))
        return;
    event->timestamp = m_timestamp_buffer[index];
    event->invalid = m_invalid_buffer[index];
    event->temperature = ((float)m_temperature_buffer[index] / 4);
}

void ICACHE_FLASH_ATTR Max6675::getSensor(sensor_t *sensor)
{
    os_memset(sensor, 0, sizeof(sensor_t));
    os_strncpy(sensor->name, "MAX6675", 7);
    sensor->sensor_id = m_id;
    sensor->type = SENSOR_TYPE_TEMPERATURE;
    sensor->max_value = 1024.0;
    sensor->min_value = 0.0;
    sensor->resolution = 0.25;
    sensor->min_delay = 85000L;
}