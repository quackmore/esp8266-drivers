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

#include "espbot_diagnostic.hpp"
#include "espbot_global.hpp"
#include "espbot_timedate.hpp"
#include "library.hpp"
#include "library_event_codes.h"
#include "library_max6675.hpp"

static void max6675_read_completed(Max6675 *max6675_ptr)
{
    int cur_pos;
    if (max6675_ptr->_buffer_idx == (max6675_ptr->_max_buffer_size - 1))
        cur_pos = 0;
    else
        cur_pos = max6675_ptr->_buffer_idx + 1;
    // check if the reading is valid
    // thermocouple disconnected => bit 2 is high
    if (max6675_ptr->_data & 0x0004)
    {
        esp_diag.error(MAX6675_THERMOCOUPLE_DISCONNECTED, (max6675_ptr->_so));
        ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] thermocouple disconnected\n",
               max6675_ptr->_cs,
               max6675_ptr->_sck,
               max6675_ptr->_so);
        // done with reading
        max6675_ptr->_reading_ongoing = false;
        // still something to do if it was a force reading
        if (max6675_ptr->_force_reading)
        {
            max6675_ptr->_force_reading = false;
            // restart polling
            os_timer_disarm(&(max6675_ptr->_poll_timer));
            if (max6675_ptr->_poll_interval > 0)
                os_timer_arm(&(max6675_ptr->_poll_timer), max6675_ptr->_poll_interval, 1);
            // actually there was no reading but anyway ...
            if (max6675_ptr->_force_reading_cb)
                max6675_ptr->_force_reading_cb(max6675_ptr->_force_reading_param);
        }
        return;
    }
    // set the value as valid
    max6675_ptr->_invalid_buffer[cur_pos] = false;
    // set the timestamp
    max6675_ptr->_timestamp_buffer[cur_pos] = esp_time.get_timestamp();
    // set the value bits: 12 bits from 3 to 14
    max6675_ptr->_temperature_buffer[cur_pos] = (max6675_ptr->_data >> 3) & 0x0FFF;
    // update the buffer position
    if (max6675_ptr->_buffer_idx == (max6675_ptr->_max_buffer_size - 1))
        max6675_ptr->_buffer_idx = 0;
    else
        max6675_ptr->_buffer_idx++;
    // done with reading
    max6675_ptr->_reading_ongoing = false;
    // still something to do if it was a force reading
    if (max6675_ptr->_force_reading)
    {
        max6675_ptr->_force_reading = false;
        // restart polling
        os_timer_disarm(&(max6675_ptr->_poll_timer));
        if (max6675_ptr->_poll_interval > 0)
            os_timer_arm(&(max6675_ptr->_poll_timer), max6675_ptr->_poll_interval, 1);

        if (max6675_ptr->_force_reading_cb)
            max6675_ptr->_force_reading_cb(max6675_ptr->_force_reading_param);
    }
}

static void max6675_read_bit(void *param)
{
    Max6675 *max6675_ptr = (Max6675 *)param;

    if (max6675_ptr->_bit_counter < 32)
    {
        // CS  _                                 _
        //      |_____________________ .._______|
        // SCK   _   _   _   _   _   _ ..  _   _
        //     _| |_| |_| |_| |_| |_| |.._| |_| |_
        //
        // current bit reading
        if (((max6675_ptr->_bit_counter) % 2) == 0)
        {
            // on even counter set SCK high and read SO
            GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->_sck), ESPBOT_HIGH);
            int bit_value = GPIO_INPUT_GET(gpio_NUM(max6675_ptr->_so));
            char bit_idx = max6675_ptr->_bit_counter / 2;
            if (bit_value)
                max6675_ptr->_data |= (0x01 << (15 - bit_idx));
        }
        else
        {
            // on odd counter set SCK low
            GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->_sck), ESPBOT_LOW);
        }
        // setup for next bit reading
        max6675_ptr->_bit_counter++;
        os_timer_disarm(&(max6675_ptr->_read_timer));
        os_timer_setfn(&(max6675_ptr->_read_timer), max6675_read_bit, (void *)max6675_ptr);
        os_timer_arm(&(max6675_ptr->_read_timer), 5, 0);
    }
    else
    {
        // reading completed
        // set SCK low
        GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->_sck), ESPBOT_LOW);
        // set CS high
        GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->_cs), ESPBOT_HIGH);
        max6675_read_completed(max6675_ptr);
    }
}

static void max6675_read(Max6675 *max6675_ptr)
{
    max6675_ptr->_reading_ongoing = true;
    // configure CS as output and set it LOW
    PIN_FUNC_SELECT(gpio_MUX(max6675_ptr->_cs), gpio_FUNC(max6675_ptr->_cs));
    GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->_cs), ESPBOT_LOW);
    // configure SCK as output and set it LOW
    PIN_FUNC_SELECT(gpio_MUX(max6675_ptr->_sck), gpio_FUNC(max6675_ptr->_sck));
    GPIO_OUTPUT_SET(gpio_NUM(max6675_ptr->_sck), ESPBOT_LOW);
    // configure SO as input
    PIN_FUNC_SELECT(gpio_MUX(max6675_ptr->_so), gpio_FUNC(max6675_ptr->_so));
    GPIO_DIS_OUTPUT(gpio_NUM(max6675_ptr->_so));
    // clear current readings
    max6675_ptr->_data = 0;
    max6675_ptr->_bit_counter = 0;
    // start the SCK sequence and SO reading
    os_timer_disarm(&(max6675_ptr->_read_timer));
    os_timer_setfn(&(max6675_ptr->_read_timer), max6675_read_bit, (void *)max6675_ptr);
    os_timer_arm(&(max6675_ptr->_read_timer), 5, 0);
}

Max6675::Max6675(int cs_pin,
                                   int sck_pin,
                                   int so_pin,
                                   int id,
                                   int poll_interval,
                                   int buffer_length)
{
    // init variables
    int idx;
    _cs = cs_pin;
    _sck = sck_pin;
    _so = so_pin;
    _id = id;

    _poll_interval = poll_interval;
    _max_buffer_size = buffer_length;

    _temperature_buffer = new int[_max_buffer_size];
    if (_temperature_buffer == NULL)
    {
        esp_diag.error(MAX6675_HEAP_EXHAUSTED, (_max_buffer_size * sizeof(int)));
        ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] heap exhausted %d",
              _cs,
              _sck,
              _so,
              (_max_buffer_size * sizeof(int)));
        return;
    }
    for (idx = 0; idx < _max_buffer_size; idx++)
        _temperature_buffer[idx] = 0;
    _timestamp_buffer = new uint32_t[_max_buffer_size];
    if (_timestamp_buffer == NULL)
    {
        esp_diag.error(MAX6675_HEAP_EXHAUSTED, (_max_buffer_size * sizeof(uint32_t)));
        ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] heap exhausted %d",
              _cs,
              _sck,
              _so,
              (_max_buffer_size * sizeof(uint32_t)));
        return;
    }
    for (idx = 0; idx < _max_buffer_size; idx++)
        _timestamp_buffer[idx] = 0;
    _invalid_buffer = new bool[_max_buffer_size];
    if (_invalid_buffer == NULL)
    {
        esp_diag.error(MAX6675_HEAP_EXHAUSTED, (_max_buffer_size * sizeof(bool)));
        ERROR("MAX6675 [CS-D%d] [SCK-D%d] [SO-D%d] heap exhausted %d",
              _cs,
              _sck,
              _so,
              (_max_buffer_size * sizeof(bool)));
        return;
    }
    for (idx = 0; idx < _max_buffer_size; idx++)
        _invalid_buffer[idx] = true;
    _buffer_idx = 0;

    _force_reading = false;
    _reading_ongoing = false;

    // set CS high
    PIN_FUNC_SELECT(gpio_MUX(_cs), gpio_FUNC(_cs));
    GPIO_OUTPUT_SET(gpio_NUM(_cs), ESPBOT_HIGH);
    // start polling
    os_timer_disarm(&_poll_timer);
    os_timer_setfn(&_poll_timer, (os_timer_func_t *)max6675_read, this);
    if (_poll_interval > 0)
        os_timer_arm(&_poll_timer, _poll_interval, 1);
}

Max6675::~Max6675()
{
    os_timer_disarm(&_poll_timer);
    if (_temperature_buffer)
        delete[] _temperature_buffer;
    if (_timestamp_buffer)
        delete[] _timestamp_buffer;
    if (_invalid_buffer)
        delete[] _invalid_buffer;
}

int Max6675::get_max_events_count(void)
{
    return _max_buffer_size;
}

void Max6675::force_reading(void (*callback)(void *), void *param)
{
    // if the class was not properly allocated exit
    if ((_timestamp_buffer == NULL) ||
        (_invalid_buffer == NULL) ||
        (_temperature_buffer == NULL))
        return;
    _force_reading_cb = callback;
    _force_reading_param = param;
    _force_reading = true;
    // in case a reading is ongoing do nothing
    // else stop the polling timer and force a reading start
    if (!_reading_ongoing)
    {
        // stop_polling
        os_timer_disarm(&_poll_timer);
        max6675_read(this);
    }
}

void Max6675::getEvent(sensors_event_t *event, int idx)
{
    os_memset(event, 0, sizeof(sensors_event_t));
    event->sensor_id = _id;
    event->type = SENSOR_TYPE_TEMPERATURE;
    // find the idx element
    int index = _buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = _max_buffer_size - 1;
        idx--;
    }
    // if the class was not properly allocated exit
    if ((_timestamp_buffer == NULL) || (_invalid_buffer == NULL) || (_temperature_buffer == NULL))
        return;
    event->timestamp = _timestamp_buffer[index];
    event->invalid = _invalid_buffer[index];
    event->temperature = ((float)_temperature_buffer[index] / 4);
}

void Max6675::getSensor(sensor_t *sensor)
{
    os_memset(sensor, 0, sizeof(sensor_t));
    os_strncpy(sensor->name, f_str("MAX6675"), 7);
    sensor->sensor_id = _id;
    sensor->type = SENSOR_TYPE_TEMPERATURE;
    sensor->max_value = 1024.0;
    sensor->min_value = 0.0;
    sensor->resolution = 0.25;
    sensor->min_delay = 85000L;
}