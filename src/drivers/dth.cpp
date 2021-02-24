/* DHT drivers

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
#include "drivers_do_sequence.h"
#include "drivers_di_sequence.h"
}

#include "espbot_diagnostic.hpp"
#include "espbot_timedate.hpp"
#include "drivers.hpp"
#include "drivers_dht.hpp"
#include "drivers_event_codes.h"

// DEBUG PROFILING
// static uint32 dht_start_sequence;
// static uint32 dht_start_sequence_completed;

static void dht_reading_completed(void *param)
{
    Dht *dht_ptr = (Dht *)param;
    if ((dht_ptr->_dht_in_sequence)->ended_by_timeout)
    {
        dia_error_evnt(DHT_READING_TIMEOUT, (dht_ptr->_dht_in_sequence)->current_pulse);
        ERROR("dht reading timeout D%d %d samples acquired", dht_ptr->_pin, (dht_ptr->_dht_in_sequence)->current_pulse);
        seq_di_clear(dht_ptr->_dht_in_sequence);
        // calculate buffer position
        // don't update the buffer position, someone could be reading ...
        int cur_pos;
        if (dht_ptr->_buffer_idx == (dht_ptr->_max_buffer_size - 1))
            cur_pos = 0;
        else
            cur_pos = dht_ptr->_buffer_idx + 1;
        // insert an invalid event
        dht_ptr->_invalid_buffer[cur_pos] = true;
        dht_ptr->_timestamp_buffer[cur_pos] = timedate_get_timestamp();
        dht_ptr->_temperature_buffer[cur_pos] = 0;
        dht_ptr->_humidity_buffer[cur_pos] = 0;
        // update the buffer position
        if (dht_ptr->_buffer_idx == (dht_ptr->_max_buffer_size - 1))
            dht_ptr->_buffer_idx = 0;
        else
            dht_ptr->_buffer_idx++;
        // done with reading
        dht_ptr->_reading_ongoing = false;
        // still something to do if it was a force reading
        if (dht_ptr->_force_reading)
        {
            dht_ptr->_force_reading = false;
            // restart polling
            os_timer_disarm(&(dht_ptr->_poll_timer));
            if (dht_ptr->_poll_interval > 0)
                os_timer_arm(&(dht_ptr->_poll_timer), dht_ptr->_poll_interval, 1);
            // actually there was no reading but anyway ...
            if (dht_ptr->_force_reading_cb)
                dht_ptr->_force_reading_cb(dht_ptr->_force_reading_param);
        }
        return;
    }
    else
    {
        // check "get ready pulse" to send data into read sequence
        uint32 pulse_duration = get_di_seq_pulse_duration(dht_ptr->_dht_in_sequence, 0);
        if ((pulse_duration < 70) || (pulse_duration > 90))
            // ERROR("DHT [D%d] reading: missing "get ready pulse"", dht_ptr->_pin);
            // nah, on second thougth why you have to mind about a correct 'get ready pulse'
            // just let it trace as an irrilevant event
            TRACE("dht reading D%d missing the get ready pulse", dht_ptr->_pin);
        // convert read sequence to _data
        int idx;
        char bit_idx, byte_idx;
        for (idx = 0; idx < 5; idx++)
            dht_ptr->_data[idx] = 0;
        bool invalid_data = false;
        for (idx = 2; idx < 81; idx = idx + 2)
        {
            pulse_duration = get_di_seq_pulse_duration(dht_ptr->_dht_in_sequence, idx);
            bit_idx = ((idx - 2) / 2) % 8;
            byte_idx = ((idx - 2) / 2) / 8;
            // found a 0 => do nothing
            if ((15 < pulse_duration) && (pulse_duration < 38))
                continue;
            // found a 1
            if ((60 < pulse_duration) && (pulse_duration < 84))
            {
                dht_ptr->_data[byte_idx] |= (0x01 << (7 - bit_idx));
                continue;
            }
            // now what the heck is this? let's try a guess
            // don't mark data as invalid yet, wait for checksum verification
            // invalid_data = true;
            // ERROR("DHT [D%d] reading: cannot understand bit (length = %d us)", dht_ptr->_pin, pulse_duration);
            if (pulse_duration > 49)
                dht_ptr->_data[byte_idx] |= (0x01 << (7 - bit_idx));
        }
        // calculate buffer position
        // don't update the buffer position, someone could be reading ...
        int cur_pos;
        if (dht_ptr->_buffer_idx == (dht_ptr->_max_buffer_size - 1))
            cur_pos = 0;
        else
            cur_pos = dht_ptr->_buffer_idx + 1;
        // check the checksum
        uint8_t checksum = dht_ptr->_data[0] + dht_ptr->_data[1] + dht_ptr->_data[2] + dht_ptr->_data[3];
        if (checksum != dht_ptr->_data[4])
        {
            dia_error_evnt(DHT_READING_CHECKSUM_ERR, dht_ptr->_pin);
            ERROR("dht reading D%d checksum error", dht_ptr->_pin);
            invalid_data = true;
        }
        // set the invalid data flag
        if (invalid_data)
            dht_ptr->_invalid_buffer[cur_pos] = true;
        else
            dht_ptr->_invalid_buffer[cur_pos] = false;
        // get the timestamp
        dht_ptr->_timestamp_buffer[cur_pos] = timedate_get_timestamp();
        // convert _data to buffer values
        // temperature
        switch (dht_ptr->_type)
        {
        case DHT11:
            dht_ptr->_temperature_buffer[cur_pos] = dht_ptr->_data[2];
            break;
        case DHT22:
        case DHT21:
            dht_ptr->_temperature_buffer[cur_pos] = dht_ptr->_data[2] & 0x7F;
            dht_ptr->_temperature_buffer[cur_pos] *= 256;
            dht_ptr->_temperature_buffer[cur_pos] += dht_ptr->_data[3];
            if (dht_ptr->_data[2] & 0x80)
                dht_ptr->_temperature_buffer[cur_pos] *= -1;
            break;
        }
        // humidity
        switch (dht_ptr->_type)
        {
        case DHT11:
            dht_ptr->_humidity_buffer[cur_pos] = dht_ptr->_data[0];
            break;
        case DHT22:
        case DHT21:
            dht_ptr->_humidity_buffer[cur_pos] = dht_ptr->_data[0];
            dht_ptr->_humidity_buffer[cur_pos] *= 256;
            dht_ptr->_humidity_buffer[cur_pos] += dht_ptr->_data[1];
            break;
        }
        // update the buffer position
        if (dht_ptr->_buffer_idx == (dht_ptr->_max_buffer_size - 1))
            dht_ptr->_buffer_idx = 0;
        else
            dht_ptr->_buffer_idx++;
        seq_di_clear(dht_ptr->_dht_in_sequence);
        // DEBUG
        // os_printf("DHT starting sequence took %d us\n", (dht_start_sequence_completed - dht_start_sequence));
        // os_printf("DHT temperature: %d\n", dht_ptr->_temperature_buffer[cur_pos]);
        // os_printf("DHT humidity   : %d\n", dht_ptr->_humidity_buffer[cur_pos]);
    }
    // done with reading
    dht_ptr->_reading_ongoing = false;
    // still something to do if it was a force reading
    if (dht_ptr->_force_reading)
    {
        dht_ptr->_force_reading = false;
        // restart polling
        os_timer_disarm(&(dht_ptr->_poll_timer));
        if (dht_ptr->_poll_interval > 0)
            os_timer_arm(&(dht_ptr->_poll_timer), dht_ptr->_poll_interval, 1);

        if (dht_ptr->_force_reading_cb)
            dht_ptr->_force_reading_cb(dht_ptr->_force_reading_param);
    }
}

static void IRAM dht_start_completed(void *param)
{
    // DEBUG
    // dht_start_sequence_completed = system_get_time();
    Dht *dht_ptr = (Dht *)param;
    // start reading from DHT
    // configure Dx as input and set pullup
    PIN_FUNC_SELECT(gpio_MUX(dht_ptr->_pin), gpio_FUNC(dht_ptr->_pin));
    PIN_PULLUP_EN(gpio_MUX(dht_ptr->_pin));
    GPIO_DIS_OUTPUT(gpio_NUM(dht_ptr->_pin));

    read_di_sequence(dht_ptr->_dht_in_sequence);

    // free output sequence
    // free_do_seq(dht_ptr->_dht_out_sequence);
}

static void IRAM dht_read(Dht *dht_ptr)
{
    dht_ptr->_reading_ongoing = true;
    // configure Dx as output and set it High
    PIN_FUNC_SELECT(gpio_MUX(dht_ptr->_pin), gpio_FUNC(dht_ptr->_pin));
    GPIO_OUTPUT_SET(gpio_NUM(dht_ptr->_pin), ESPBOT_HIGH);
    // configure start sequence (1 pulse)
    // High ____                    ____ 20 us ___
    // Low      |_____ 1,5 ms _____|
    if (!dht_ptr->_dht_out_sequence)
    {
        dht_ptr->_dht_out_sequence = new_do_seq(gpio_NUM(dht_ptr->_pin), 2);
        if (dht_ptr->_dht_out_sequence == NULL)
        {
            dia_error_evnt(DHT_READ_HEAP_EXHAUSTED, sizeof(struct do_seq));
            ERROR("dht_read heap exhausted %d", sizeof(struct do_seq));
            return;
        }
        set_do_seq_cb(dht_ptr->_dht_out_sequence, dht_start_completed, (void *)dht_ptr, direct);
        out_seq_add(dht_ptr->_dht_out_sequence, ESPBOT_LOW, 1500);
        out_seq_add(dht_ptr->_dht_out_sequence, ESPBOT_HIGH, 10);
    }
    // prepare input sequence (82 pulses)
    if (!dht_ptr->_dht_in_sequence)
    {
        dht_ptr->_dht_in_sequence = new_di_seq(gpio_NUM(dht_ptr->_pin), 82, 100, TIMEOUT_MS);
        if (dht_ptr->_dht_in_sequence == NULL)
        {
            dia_error_evnt(DHT_READ_HEAP_EXHAUSTED, sizeof(struct do_seq));
            ERROR("dht_read heap exhausted %d", sizeof(struct di_seq));
            return;
        }
        set_di_seq_cb(dht_ptr->_dht_in_sequence, dht_reading_completed, (void *)dht_ptr, task);
    }
    // Send start sequence
    // DEBUG
    // dht_start_sequence = system_get_time();
    exe_do_seq_us(dht_ptr->_dht_out_sequence);
}

Dht::Dht(int pin,
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
        _data[idx] = 0;
    _pin = pin;
    _type = type;
    _poll_interval = poll_interval;
    _max_buffer_size = buffer_length;
    // data buffers
    _temperature_buffer = new int[_max_buffer_size];
    if (_temperature_buffer == NULL)
    {
        dia_error_evnt(DHT_HEAP_EXHAUSTED, (_max_buffer_size * sizeof(int)));
        ERROR("Dht heap exhausted %d", (_max_buffer_size * sizeof(int)));
        return;
    }
    for (idx = 0; idx < _max_buffer_size; idx++)
        _temperature_buffer[idx] = 0;
    _humidity_buffer = new int[_max_buffer_size];
    if (_humidity_buffer == NULL)
    {
        dia_error_evnt(DHT_HEAP_EXHAUSTED, (_max_buffer_size * sizeof(int)));
        ERROR("Dht heap exhausted %d", (_max_buffer_size * sizeof(int)));
        return;
    }
    for (idx = 0; idx < _max_buffer_size; idx++)
        _humidity_buffer[idx] = 0;
    _invalid_buffer = new bool[_max_buffer_size];
    if (_invalid_buffer == NULL)
    {
        dia_error_evnt(DHT_HEAP_EXHAUSTED, (_max_buffer_size * sizeof(int)));
        ERROR("Dht heap exhausted %d", (_max_buffer_size * sizeof(bool)));
        return;
    }
    for (idx = 0; idx < _max_buffer_size; idx++)
        _invalid_buffer[idx] = true;
    _timestamp_buffer = new uint32_t[_max_buffer_size];
    if (_timestamp_buffer == NULL)
    {
        dia_error_evnt(DHT_HEAP_EXHAUSTED, (_max_buffer_size * sizeof(int)));
        ERROR("Dht heap exhausted %d", (_max_buffer_size * sizeof(uint32_t)));
        return;
    }
    for (idx = 0; idx < _max_buffer_size; idx++)
        _timestamp_buffer[idx] = 0;
    _buffer_idx = 0;

    _dht_out_sequence = NULL;
    _dht_in_sequence = NULL;
    _force_reading = false;
    _force_reading_cb = NULL;
    _force_reading_param = NULL;
    _reading_ongoing = false;
    // setup polling
    os_timer_disarm(&_poll_timer);
    os_timer_setfn(&_poll_timer, (os_timer_func_t *)dht_read, this);
    if (_poll_interval > 0)
        os_timer_arm(&_poll_timer, _poll_interval, 1);
}

Dht::~Dht()
{
    os_timer_disarm(&_poll_timer);
    if (_dht_in_sequence)
        free_di_seq(_dht_in_sequence);
    if (_dht_out_sequence)
        free_do_seq(_dht_out_sequence);
    if (_temperature_buffer)
        delete[] _temperature_buffer;
    if (_humidity_buffer)
        delete[] _humidity_buffer;
    if (_invalid_buffer)
        delete[] _invalid_buffer;
    if (_timestamp_buffer)
        delete[] _timestamp_buffer;
}

Dht::Temperature::Temperature(Dht *parent, int id)
{
    _parent = parent;
    _id = id;
}

Dht::Temperature::~Temperature()
{
}

int Dht::Temperature::get_max_events_count(void)
{
    return _parent->_max_buffer_size;
}

void Dht::Temperature::force_reading(void (*callback)(void *), void *param)
{
    // if the class was not properly allocated exit
    if ((_parent->_timestamp_buffer == NULL) ||
        (_parent->_invalid_buffer == NULL) ||
        (_parent->_temperature_buffer == NULL))
        return;
    _parent->_force_reading_cb = callback;
    _parent->_force_reading_param = param;
    _parent->_force_reading = true;
    // in case a reading is ongoing do nothing
    // else stop the polling timer and force a reading start
    if (!_parent->_reading_ongoing)
    {
        // stop_polling
        os_timer_disarm(&_parent->_poll_timer);
        dht_read(_parent);
    }
}

void Dht::Temperature::getEvent(sensors_event_t *event, int idx)
{
    os_memset(event, 0, sizeof(sensors_event_t));
    event->sensor_id = _id;
    event->type = SENSOR_TYPE_TEMPERATURE;
    // find the idx element
    int index = _parent->_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = _parent->_max_buffer_size - 1;
        idx--;
    }
    // if the class was not properly allocated exit
    if ((_parent->_timestamp_buffer == NULL) ||
        (_parent->_invalid_buffer == NULL) ||
        (_parent->_temperature_buffer == NULL))
        return;
    event->timestamp = _parent->_timestamp_buffer[index];
    event->invalid = _parent->_invalid_buffer[index];
    switch (_parent->_type)
    {
    case DHT11:
        event->temperature = (float)_parent->_temperature_buffer[index];
    case DHT22:
    case DHT21:
        event->temperature = ((float)_parent->_temperature_buffer[index] * 0.1);
    }
}

void Dht::Temperature::getSensor(sensor_t *sensor)
{
    os_memset(sensor, 0, sizeof(sensor_t));
    sensor->sensor_id = _id;
    sensor->type = SENSOR_TYPE_TEMPERATURE;
    switch (_parent->_type)
    {
    case DHT11:
        os_strncpy(sensor->name, f_str("DHT11"), 6);
        sensor->max_value = 50.0;
        sensor->min_value = 0.0;
        sensor->resolution = 2.0;
        sensor->min_delay = 1000000L; // 1 second (in microseconds)
        break;
    case DHT21:
        os_strncpy(sensor->name, f_str("DHT21"), 6);
        sensor->max_value = 80.0;
        sensor->min_value = -40.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    case DHT22:
        os_strncpy(sensor->name, f_str("DHT22"), 6);
        sensor->max_value = 125.0;
        sensor->min_value = -40.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    default:
        // just in case
        os_strncpy(sensor->name, f_str("Unknown"), 8);
        sensor->max_value = 0.0;
        sensor->min_value = 0.0;
        sensor->resolution = 0.0;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    }
}

Dht::Humidity::Humidity(Dht *parent, int id)
{
    _parent = parent;
    _id = id;
}

Dht::Humidity::~Humidity()
{
}

int Dht::Humidity::get_max_events_count(void)
{
    return _parent->_max_buffer_size;
}

void Dht::Humidity::force_reading(void (*callback)(void *), void *param)
{
    _parent->_force_reading_cb = callback;
    _parent->_force_reading_param = param;
    _parent->_force_reading = true;
    // in case a reading is ongoing do nothing
    // else stop the polling timer and force a reading start
    if (!_parent->_reading_ongoing)
    {
        // stop_polling
        os_timer_disarm(&_parent->_poll_timer);
        dht_read(_parent);
    }
}

void Dht::Humidity::getEvent(sensors_event_t *event, int idx)
{
    os_memset(event, 0, sizeof(sensors_event_t));
    event->sensor_id = _id;
    event->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    // find the idx element
    int index = _parent->_buffer_idx;
    while (idx > 0)
    {
        index = index - 1;
        if (index < 0)
            index = _parent->_max_buffer_size - 1;
        idx--;
    }
    event->timestamp = _parent->_timestamp_buffer[index];
    event->invalid = _parent->_invalid_buffer[index];
    switch (_parent->_type)
    {
    case DHT11:
        event->relative_humidity = (float)_parent->_humidity_buffer[index];
    case DHT22:
    case DHT21:
        event->relative_humidity = ((float)_parent->_humidity_buffer[index] * 0.1);
    }
}

void Dht::Humidity::getSensor(sensor_t *sensor)
{
    os_memset(sensor, 0, sizeof(sensor_t));
    sensor->sensor_id = _id;
    sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    switch (_parent->_type)
    {
    case DHT11:
        os_strncpy(sensor->name, f_str("DHT11"), 6);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 2.0;
        sensor->min_delay = 1000000L; // 1 second (in microseconds)
        break;
    case DHT21:
        os_strncpy(sensor->name, f_str("DHT21"), 6);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    case DHT22:
        os_strncpy(sensor->name, f_str("DHT22"), 6);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 0.1;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    default:
        // just in case
        os_strncpy(sensor->name, f_str("Unknown"), 8);
        sensor->max_value = 100.0;
        sensor->min_value = 0.0;
        sensor->resolution = 2.0;
        sensor->min_delay = 2000000L; // 2 seconds (in microseconds)
        break;
    }
}