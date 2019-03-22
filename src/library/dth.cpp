/* DHT library

MIT license
written by Adafruit Industries
*/
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
}

#include "library.hpp"
#include "library_dht.hpp"

static void ICACHE_FLASH_ATTR dht_reading_completed(void *param)
{
    Dht *dht_ptr = (Dht *)param;
    if ((dht_ptr->m_dht_in_sequence)->ended_by_timeout)
    {
        PRINT_ERROR("DHT [D%d] reading timeout, %d samples acquired\n", dht_ptr->m_pin, (dht_ptr->m_dht_in_sequence)->current_pulse);
        seq_di_clear(dht_ptr->m_dht_in_sequence);
        // done with reading
        dht_ptr->m_reading_ongoing = false;
        // still something to do if it was a force reading
        if (dht_ptr->m_force_reading)
        {
            dht_ptr->m_force_reading = false;
            // restart polling
            os_timer_disarm(&(dht_ptr->m_poll_timer));
            if (dht_ptr->m_poll_interval > 0)
                os_timer_arm(&(dht_ptr->m_poll_timer), dht_ptr->m_poll_interval, 1);
            // actually there was no reading but anyway ...
            dht_ptr->m_force_reading_cb(dht_ptr->m_force_reading_param);
        }
        return;
    }
    else
    {
        // check preparation to send data into read sequence
        uint32 pulse_duration = get_di_seq_pulse_duration(dht_ptr->m_dht_in_sequence, 0);
        if ((pulse_duration < 70) || (pulse_duration > 90))
            // PRINT_ERROR("DHT [D%d] reading: missing preparation to send data\n", dht_ptr->m_pin);
            // nah, on second thougth why you have to mind about a correct 'preparation to send data'
            // just let it trace as an irrilevant eent
            PRINT_TRACE("DHT [D%d] reading: missing preparation to send data\n", dht_ptr->m_pin);
        // convert read sequence to m_data
        int idx;
        char bit_idx, byte_idx;
        for (idx = 0; idx < 5; idx++)
            dht_ptr->m_data[idx] = 0;
        bool invalid_data = false;
        for (idx = 2; idx < 81; idx = idx + 2)
        {
            pulse_duration = get_di_seq_pulse_duration(dht_ptr->m_dht_in_sequence, idx);
            bit_idx = ((idx - 2) / 2) % 8;
            byte_idx = ((idx - 2) / 2) / 8;
            // found a 0 => do nothing
            if ((15 < pulse_duration) && (pulse_duration < 38))
                continue;
            // found a 1
            if ((60 < pulse_duration) && (pulse_duration < 84))
            {
                dht_ptr->m_data[byte_idx] |= (0x01 << (7 - bit_idx));
                continue;
            }
            // now what the heck is this? let's try a guess
            // don't mark data as invalid yet, wait for checksum verification
            // invalid_data = true;
            // PRINT_ERROR("DHT [D%d] reading: cannot understand bit (length = %d us)\n", dht_ptr->m_pin, pulse_duration);
            if (pulse_duration > 49)
                dht_ptr->m_data[byte_idx] |= (0x01 << (7 - bit_idx));
        }
        // calculate buffer position
        // don't update the buffer position, someone could be reading ...
        int cur_pos;
        if (dht_ptr->m_buffer_idx == (dht_ptr->m_max_buffer_size - 1))
            cur_pos = 0;
        else
            cur_pos = dht_ptr->m_buffer_idx + 1;
        // check the checksum
        uint8_t checksum = dht_ptr->m_data[0] + dht_ptr->m_data[1] + dht_ptr->m_data[2] + dht_ptr->m_data[3];
        if (checksum != dht_ptr->m_data[4])
        {
            PRINT_ERROR("DHT [D%d] reading: checksum error\n", dht_ptr->m_pin);
            invalid_data = true;
        }
        // set the invalid data flag
        if (invalid_data)
            dht_ptr->m_invalid_buffer[cur_pos] = true;
        else
            dht_ptr->m_invalid_buffer[cur_pos] = false;
        // get the timestamp
        dht_ptr->m_timestamp_buffer[cur_pos] = getTimestamp();
        // convert m_data to buffer values
        // temperature
        switch (dht_ptr->m_type)
        {
        case DHT11:
            dht_ptr->m_temperature_buffer[cur_pos] = dht_ptr->m_data[2];
            break;
        case DHT22:
        case DHT21:
            dht_ptr->m_temperature_buffer[cur_pos] = dht_ptr->m_data[2] & 0x7F;
            dht_ptr->m_temperature_buffer[cur_pos] *= 256;
            dht_ptr->m_temperature_buffer[cur_pos] += dht_ptr->m_data[3];
            if (dht_ptr->m_data[2] & 0x80)
                dht_ptr->m_temperature_buffer[cur_pos] *= -1;
            break;
        }
        // humidity
        switch (dht_ptr->m_type)
        {
        case DHT11:
            dht_ptr->m_humidity_buffer[cur_pos] = dht_ptr->m_data[0];
            break;
        case DHT22:
        case DHT21:
            dht_ptr->m_humidity_buffer[cur_pos] = dht_ptr->m_data[0];
            dht_ptr->m_humidity_buffer[cur_pos] *= 256;
            dht_ptr->m_humidity_buffer[cur_pos] += dht_ptr->m_data[1];
            break;
        }
        // update the buffer position
        if (dht_ptr->m_buffer_idx == (dht_ptr->m_max_buffer_size - 1))
            dht_ptr->m_buffer_idx = 0;
        else
            dht_ptr->m_buffer_idx++;
        seq_di_clear(dht_ptr->m_dht_in_sequence);
    }
    // done with reading
    dht_ptr->m_reading_ongoing = false;
    // still something to do if it was a force reading
    if (dht_ptr->m_force_reading)
    {
        dht_ptr->m_force_reading = false;
        // restart polling
        os_timer_disarm(&(dht_ptr->m_poll_timer));
        if (dht_ptr->m_poll_interval > 0)
            os_timer_arm(&(dht_ptr->m_poll_timer), dht_ptr->m_poll_interval, 1);

        dht_ptr->m_force_reading_cb(dht_ptr->m_force_reading_param);
    }
}

static void dht_start_completed(void *param)
{
    Dht *dht_ptr = (Dht *)param;
    // start reading from DHT
    // configure Dx as input and set pullup
    PIN_FUNC_SELECT(gpio_MUX(dht_ptr->m_pin), gpio_FUNC(dht_ptr->m_pin));
    PIN_PULLUP_EN(gpio_MUX(dht_ptr->m_pin));
    GPIO_DIS_OUTPUT(gpio_NUM(dht_ptr->m_pin));

    read_di_sequence(dht_ptr->m_dht_in_sequence);

    // free output sequence
    // free_do_seq(dht_ptr->m_dht_out_sequence);
}

static void dht_read(Dht *dht_ptr)
{
    dht_ptr->m_reading_ongoing = true;
    // configure Dx as output and set it High
    PIN_FUNC_SELECT(gpio_MUX(dht_ptr->m_pin), gpio_FUNC(dht_ptr->m_pin));
    GPIO_OUTPUT_SET(gpio_NUM(dht_ptr->m_pin), ESPBOT_HIGH);
    // configure start sequence (1 pulse)
    // High ____                    ____ 20 us ___
    // Low      |_____ 1,5 ms _____|
    if (!dht_ptr->m_dht_out_sequence)
    {
        dht_ptr->m_dht_out_sequence = new_do_seq(gpio_NUM(dht_ptr->m_pin), 2);
        set_do_seq_cb(dht_ptr->m_dht_out_sequence, dht_start_completed, (void *)dht_ptr, direct);
        out_seq_add(dht_ptr->m_dht_out_sequence, ESPBOT_LOW, 1500);
        out_seq_add(dht_ptr->m_dht_out_sequence, ESPBOT_HIGH, 10);
    }
    // prepare input sequence (82 pulses)
    if (!dht_ptr->m_dht_in_sequence)
    {
        dht_ptr->m_dht_in_sequence = new_di_seq(ESPBOT_D2_NUM, 82, 100, TIMEOUT_MS);
        set_di_seq_cb(dht_ptr->m_dht_in_sequence, dht_reading_completed, (void *)dht_ptr, task);
    }
    // Send start sequence
    exe_do_seq_us(dht_ptr->m_dht_out_sequence);
}

ICACHE_FLASH_ATTR Dht::Dht(int pin,
                           Dht_type type,
                           int temperature_id,
                           int humidity_id,
                           int poll_interval,
                           int buffer_length)
    : temperature(this, temperature_id),
      humidity(this, humidity_id)
{
    int idx;
    for (idx = 0; idx < 5; idx++)
        m_data[idx] = 0;
    m_pin = pin;
    m_type = type;
    m_poll_interval = poll_interval;
    m_max_buffer_size = buffer_length;
    // data buffers
    m_temperature_buffer = new int[m_max_buffer_size];
    for (idx = 0; idx < m_max_buffer_size; idx++)
        m_temperature_buffer[idx] = 0;
    m_humidity_buffer = new int[m_max_buffer_size];
    for (idx = 0; idx < m_max_buffer_size; idx++)
        m_humidity_buffer[idx] = 0;
    m_invalid_buffer = new bool[m_max_buffer_size];
    for (idx = 0; idx < m_max_buffer_size; idx++)
        m_invalid_buffer[idx] = true;
    m_timestamp_buffer = new uint32_t[m_max_buffer_size];
    for (idx = 0; idx < m_max_buffer_size; idx++)
        m_timestamp_buffer[idx] = 0;
    m_buffer_idx = 0;

    m_dht_out_sequence = NULL;
    m_dht_in_sequence = NULL;
    m_force_reading = false;
    m_force_reading_cb = NULL;
    m_force_reading_param = NULL;
    m_reading_ongoing = false;
    // setup polling
    os_timer_disarm(&m_poll_timer);
    os_timer_setfn(&m_poll_timer, (os_timer_func_t *)dht_read, this);
    if (m_poll_interval > 0)
        os_timer_arm(&m_poll_timer, m_poll_interval, 1);
}

ICACHE_FLASH_ATTR Dht::~Dht()
{
    if (m_dht_in_sequence)
        free_di_seq(m_dht_in_sequence);
    if (m_dht_out_sequence)
        free_do_seq(m_dht_out_sequence);
    delete[] m_temperature_buffer;
    delete[] m_humidity_buffer;
    delete[] m_invalid_buffer;
    delete[] m_timestamp_buffer;
}

ICACHE_FLASH_ATTR Dht::Temperature::Temperature(Dht *parent, int id)
{
    m_parent = parent;
    m_id = id;
}

ICACHE_FLASH_ATTR Dht::Temperature::~Temperature()
{
}

int ICACHE_FLASH_ATTR Dht::Temperature::get_max_events_count(void)
{
    return m_parent->m_max_buffer_size;
}

void ICACHE_FLASH_ATTR Dht::Temperature::force_reading(void (*callback)(void *), void *param)
{
    m_parent->m_force_reading_cb = callback;
    m_parent->m_force_reading_param = param;
    m_parent->m_force_reading = true;
    // in case a reading is ongoing do nothing
    // else stop the polling timer and force a reading start
    if (!m_parent->m_reading_ongoing)
    {
        // stop_polling
        os_timer_disarm(&m_parent->m_poll_timer);
        dht_read(m_parent);
    }
}

void ICACHE_FLASH_ATTR Dht::Temperature::getEvent(sensors_event_t *event, int idx)
{
    os_memset(event, 0, sizeof(sensors_event_t));
    event->sensor_id = m_id;
    event->type = SENSOR_TYPE_TEMPERATURE;
    // find the idx element
    int index = m_parent->m_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = m_parent->m_max_buffer_size - 1;
        idx--;
    }
    event->timestamp = m_parent->m_timestamp_buffer[index];
    event->invalid = m_parent->m_invalid_buffer[index];
    switch (m_parent->m_type)
    {
    case DHT11:
        event->temperature = (float)m_parent->m_temperature_buffer[index];
    case DHT22:
    case DHT21:
        event->temperature = ((float)m_parent->m_temperature_buffer[index] * 0.1);
    }
}

void ICACHE_FLASH_ATTR Dht::Temperature::getSensor(sensor_t *sensor)
{
    os_memset(sensor, 0, sizeof(sensor_t));
    sensor->sensor_id = m_id;
    sensor->type = SENSOR_TYPE_TEMPERATURE;
    switch (m_parent->m_type)
    {
    case DHT11:
        os_strncpy(sensor->name, "DHT11", 6);
        sensor->max_value = 50.0;
        sensor->min_value = 0.0;
        sensor->resolution = 2.0;
        sensor->min_delay = 1000000L; // 1 second (in microseconds)
        break;
    case DHT21:
        os_strncpy(sensor->name, "DHT21", 6);
        sensor->max_value = 80.0;
        sensor->min_value = -40.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    case DHT22:
        os_strncpy(sensor->name, "DHT22", 6);
        sensor->max_value = 125.0;
        sensor->min_value = -40.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    default:
        // just in case
        os_strncpy(sensor->name, "Unknown", 8);
        sensor->max_value = 0.0;
        sensor->min_value = 0.0;
        sensor->resolution = 0.0;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    }
}

ICACHE_FLASH_ATTR Dht::Humidity::Humidity(Dht *parent, int id)
{
    m_parent = parent;
    m_id = id;
}

ICACHE_FLASH_ATTR Dht::Humidity::~Humidity()
{
}

int ICACHE_FLASH_ATTR Dht::Humidity::get_max_events_count(void)
{
    return m_parent->m_max_buffer_size;
}

void ICACHE_FLASH_ATTR Dht::Humidity::force_reading(void (*callback)(void *), void *param)
{
    m_parent->m_force_reading_cb = callback;
    m_parent->m_force_reading_param = param;
    m_parent->m_force_reading = true;
    // in case a reading is ongoing do nothing
    // else stop the polling timer and force a reading start
    if (!m_parent->m_reading_ongoing)
    {
        // stop_polling
        os_timer_disarm(&m_parent->m_poll_timer);
        dht_read(m_parent);
    }
}

void ICACHE_FLASH_ATTR Dht::Humidity::getEvent(sensors_event_t *event, int idx)
{
    os_memset(event, 0, sizeof(sensors_event_t));
    event->sensor_id = m_id;
    event->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    // find the idx element
    int index = m_parent->m_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = m_parent->m_max_buffer_size - 1;
        idx--;
    }
    event->timestamp = m_parent->m_timestamp_buffer[index];
    event->invalid = m_parent->m_invalid_buffer[index];
    switch (m_parent->m_type)
    {
    case DHT11:
        event->relative_humidity = (float)m_parent->m_humidity_buffer[index];
    case DHT22:
    case DHT21:
        event->relative_humidity = ((float)m_parent->m_humidity_buffer[index] * 0.1);
    }
}

void ICACHE_FLASH_ATTR Dht::Humidity::getSensor(sensor_t *sensor)
{
    os_memset(sensor, 0, sizeof(sensor_t));
    sensor->sensor_id = m_id;
    sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    switch (m_parent->m_type)
    {
    case DHT11:
        os_strncpy(sensor->name, "DHT11", 6);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 2.0;
        sensor->min_delay = 1000000L; // 1 second (in microseconds)
        break;
    case DHT21:
        os_strncpy(sensor->name, "DHT21", 6);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    case DHT22:
        os_strncpy(sensor->name, "DHT22", 6);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    default:
        // just in case
        os_strncpy(sensor->name, "Unknown", 8);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 2.0;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    }
}