#include <application.h>
#include <bc_ds18b20.h>
#include <accelerometer.h>

#define BATTERY_UPDATE_INTERVAL   (5 * 60 * 1000)
#define SEND_DATA_INTERVAL        (5 * 60 * 1000)
#define MEASURE_INTERVAL              (10 * 1000)
#define DS18B20_SENSOR_COUNT 4


// LED instance
bc_led_t led;

// Button instance
bc_button_t button;

// Thermometer instance
bc_tmp112_t tmp112;

// ds18b20 library instance
static bc_ds18b20_t ds18b20;

// ds18b20 sensors array
static bc_ds18b20_sensor_t ds18b20_sensors[DS18B20_SENSOR_COUNT];

// Humidity tag instance
bc_tag_humidity_t humidity;

// Magnet Switch instance
bc_switch_t swith;

BC_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer_0, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
BC_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer_1, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
BC_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer_2, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))
BC_DATA_STREAM_FLOAT_BUFFER(sm_temperature_buffer_3, (SEND_DATA_INTERVAL / MEASURE_INTERVAL))

bc_data_stream_t sm_temperature_0;
bc_data_stream_t sm_temperature_1;
bc_data_stream_t sm_temperature_2;
bc_data_stream_t sm_temperature_3;

bc_data_stream_t *sm_temperature[] =
{
    &sm_temperature_0,
    &sm_temperature_1,
    &sm_temperature_2,
    &sm_temperature_3,
};

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    if (event == BC_BUTTON_EVENT_PRESS)
    {
        static uint16_t button_click_count = 0;

        // Pulse LED for 100 milliseconds
        bc_led_pulse(&led, 100);

        // Increment press count
        button_click_count++;

        bc_log_info("APP: Publish button press count = %u", button_click_count);

        // Publish button message on radio
        bc_radio_pub_push_button(&button_click_count);
    }
}

void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *event_param)
{
    float value;

    if (event == BC_TMP112_EVENT_UPDATE)
    {
        if (bc_tmp112_get_temperature_celsius(self, &value))
        {
            bc_radio_pub_temperature(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &value);
        }
    }
}

void battery_event_handler(bc_module_battery_event_t event, void *event_param)
{
    (void) event_param;

    float voltage;

    if (event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        if (bc_module_battery_get_voltage(&voltage))
        {
            bc_radio_pub_battery(&voltage);
        }
    }
}

void handler_ds18b20(bc_ds18b20_t *self, uint64_t device_address, bc_ds18b20_event_t event, void *event_param)
{
    (void) event_param;

    float value = NAN;

    if (event == bc_ds18b20_EVENT_UPDATE)
    {
        bc_ds18b20_get_temperature_celsius(self, device_address, &value);

        int device_index = bc_ds18b20_get_index_by_device_address(self, device_address);

        bc_log_debug("UPDATE %llx (%d) = %f", device_address, device_index, value);

        bc_data_stream_feed(sm_temperature[device_index], &value);

        static char topic[64];
        snprintf(topic, sizeof(topic), "thermometer/%" PRIx64 "/temperature", device_address);
        bc_radio_pub_float(topic, &value);
    }
}

void humidity_tag_event_handler(bc_tag_humidity_t *self, bc_tag_humidity_event_t event, void *event_param)
{
    float value;

    if (event == BC_TAG_HUMIDITY_EVENT_UPDATE)
    {
        if (bc_tag_humidity_get_humidity_percentage(self, &value))
        {
            bc_radio_pub_humidity(BC_RADIO_PUB_CHANNEL_R3_I2C0_ADDRESS_DEFAULT, &value);
        }
    }
}

int switch_cnt = 0;

void switch_event_handler(bc_switch_t *self, bc_switch_event_t event, void *event_param)
{
    if (event == BC_SWITCH_EVENT_CLOSED)
    {
        switch_cnt++;
        bc_radio_pub_int("switch_cnt", &switch_cnt);
    }
}

int adc_cntd;
float max;

void adc_measure_task(void *param)
{
    bc_adc_calibration();

    adc_cntd = 50;
    max = 0;

    bc_adc_async_measure(BC_ADC_CHANNEL_A4);

    bc_scheduler_plan_current_from_now(MEASURE_INTERVAL);
}

void adc_event_handler(bc_adc_channel_t channel, bc_adc_event_t event, void *event_param)
{
    if (event == BC_ADC_EVENT_DONE)
    {
        float result;
        float vdda_voltage;

        bc_adc_get_vdda_voltage(&vdda_voltage);

        bc_adc_async_get_voltage(channel, &result);

        float v = (vdda_voltage / 2) - (result);

        if (v > max)
        {
            max = v;
        }


        if (adc_cntd-- == 0)
        {
            float a = max * 15.f * 0.707;
            float w = a * 220 - 10.0;

            if (w < 0) w = 0;

            bc_log_debug("W %.2f", w);

            bc_radio_pub_float("watt", &w);
        }
        else
        {
            bc_adc_async_measure(BC_ADC_CHANNEL_A4);
        }

    }
}

void application_init(void)
{
    // Initialize logging
    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_ABS);

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize battery
    bc_module_battery_init();
    bc_module_battery_set_event_handler(battery_event_handler, NULL);
    bc_module_battery_set_update_interval(BATTERY_UPDATE_INTERVAL);

    // Initialize thermometer sensor on core module
    bc_tmp112_init(&tmp112, BC_I2C_I2C0, 0x49);
    bc_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    bc_tmp112_set_update_interval(&tmp112, MEASURE_INTERVAL);

    // Initialize 1-Wire temperature sensors
    bc_ds18b20_init_multiple(&ds18b20, ds18b20_sensors, DS18B20_SENSOR_COUNT, BC_DS18B20_RESOLUTION_BITS_12);
    bc_ds18b20_set_event_handler(&ds18b20, handler_ds18b20, NULL);
    bc_ds18b20_set_update_interval(&ds18b20, MEASURE_INTERVAL);

    bc_tag_humidity_init(&humidity, BC_TAG_HUMIDITY_REVISION_R3, BC_I2C_I2C0, BC_TAG_HUMIDITY_I2C_ADDRESS_DEFAULT);
    bc_tag_humidity_set_update_interval(&humidity, MEASURE_INTERVAL);
    bc_tag_humidity_set_event_handler(&humidity, humidity_tag_event_handler, NULL);

    bc_adc_init();
    bc_adc_oversampling_set(BC_ADC_CHANNEL_A4, BC_ADC_OVERSAMPLING_256);
    bc_adc_set_event_handler(BC_ADC_CHANNEL_A4, adc_event_handler, NULL);
    bc_scheduler_register(adc_measure_task, NULL, 1000);

    bc_switch_init(&swith, BC_GPIO_P9, BC_SWITCH_PULL_UP_DYNAMIC, BC_SWITCH_TYPE_NC);
    bc_switch_set_event_handler(&swith, switch_event_handler, NULL);

    bc_radio_init(BC_RADIO_MODE_NODE_SLEEPING);
    bc_radio_pairing_request("stetoskop", VERSION);

    // accelerometer_init(200);

    bc_led_pulse(&led, 2000);
}

void application_task(void)
{

}
