#include <accelerometer.h>
#include <bc_lis2dh12.h>
#include <bc_data_stream.h>
#include <bc_dice.h>
#include <bc_log.h>

#define _ACCELEROMETER_CHANGE 0.2f

BC_DATA_STREAM_FLOAT_BUFFER(_accelerometer_x_axis_buffer, ACCELEROMETER_NUMBER_OF_SAMPLES)
BC_DATA_STREAM_FLOAT_BUFFER(_accelerometer_y_axis_buffer, ACCELEROMETER_NUMBER_OF_SAMPLES)
BC_DATA_STREAM_FLOAT_BUFFER(_accelerometer_z_axis_buffer, ACCELEROMETER_NUMBER_OF_SAMPLES)

static struct
{
    bc_lis2dh12_t lis2dh12;

    bc_dice_t dice;

    struct
    {
        bc_data_stream_t x_axis;
        bc_data_stream_t y_axis;
        bc_data_stream_t z_axis;

    } data_stream;

    bc_lis2dh12_result_g_t center_g;
    bool center_g_valid;
    float angle;

} _accelerometer;

static void _accelerometer_lis2dh12_event_handler(bc_lis2dh12_t *self, bc_lis2dh12_event_t event, void *event_param);

void accelerometer_init(bc_tick_t update_interval)
{
    memset(&_accelerometer, 0, sizeof(_accelerometer));

    bc_dice_init(&_accelerometer.dice, BC_DICE_FACE_UNKNOWN);

    bc_lis2dh12_init(&_accelerometer.lis2dh12, BC_I2C_I2C0, 0x19);
    bc_lis2dh12_set_resolution(&_accelerometer.lis2dh12, BC_LIS2DH12_RESOLUTION_8BIT);
    bc_lis2dh12_set_event_handler(&_accelerometer.lis2dh12, _accelerometer_lis2dh12_event_handler, NULL);
    bc_lis2dh12_set_update_interval(&_accelerometer.lis2dh12, update_interval);

    bc_data_stream_init(&_accelerometer.data_stream.x_axis, 1, &_accelerometer_x_axis_buffer);
    bc_data_stream_init(&_accelerometer.data_stream.y_axis, 1, &_accelerometer_y_axis_buffer);
    bc_data_stream_init(&_accelerometer.data_stream.z_axis, 1, &_accelerometer_z_axis_buffer);
}

uint8_t accelerometer_get_position(void)
{
    return bc_dice_get_face(&_accelerometer.dice);
}

float accelerometer_get_angle(void)
{
    return _accelerometer.angle;
}

static void _accelerometer_lis2dh12_event_handler(bc_lis2dh12_t *self, bc_lis2dh12_event_t event, void *event_param)
{
    (void) event_param;
    float x_axis;
    float y_axis;
    float z_axis;

    if (event == BC_LIS2DH12_EVENT_UPDATE)
    {
        bc_lis2dh12_result_g_t g;

        if (!bc_lis2dh12_get_result_g(self, &g))
        {
            return;
        }

        bc_data_stream_feed(&_accelerometer.data_stream.x_axis, &g.x_axis);
        bc_data_stream_feed(&_accelerometer.data_stream.y_axis, &g.y_axis);
        bc_data_stream_feed(&_accelerometer.data_stream.z_axis, &g.z_axis);

        bc_data_stream_get_median(&_accelerometer.data_stream.x_axis, &x_axis);
        bc_data_stream_get_median(&_accelerometer.data_stream.y_axis, &y_axis);
        bc_data_stream_get_median(&_accelerometer.data_stream.z_axis, &z_axis);

        bc_dice_feed_vectors(&_accelerometer.dice, x_axis, y_axis, z_axis);

        if (_accelerometer.center_g_valid)
        {
            float angle = atan(g.y_axis / g.x_axis) - atan(_accelerometer.center_g.y_axis / _accelerometer.center_g.x_axis);

            angle *= (180.f / 3.14159265358979323846264338327950288);

            if (angle < -90)
            {
                angle = 180 + angle;
            }

            _accelerometer.angle = angle;


            bc_log_debug("dice %d", bc_dice_get_face(&_accelerometer.dice));
            bc_log_debug("angle %.2f", _accelerometer.angle);
        }
        else
        {
            if (bc_data_stream_get_counter(&_accelerometer.data_stream.x_axis) > 20)
            {
                if (!bc_data_stream_get_median(&_accelerometer.data_stream.x_axis, &_accelerometer.center_g.x_axis) ||
                    !bc_data_stream_get_median(&_accelerometer.data_stream.y_axis, &_accelerometer.center_g.y_axis) ||
                    !bc_data_stream_get_median(&_accelerometer.data_stream.z_axis, &_accelerometer.center_g.z_axis))
                {
                    return;
                }

                _accelerometer.center_g_valid = true;
            }
        }

    }
}



// void lis2dh12_event_handler(bc_lis2dh12_t *self, bc_lis2dh12_event_t event, void *event_param)
// {
//     (void) event_param;

//     if (event != BC_LIS2DH12_EVENT_ERROR)
//     {

//         bc_lis2dh12_result_g_t g;

//         if (!bc_lis2dh12_get_result_g(self, &g))
//         {
//             return;
//         }

//         if (accelerometer.center_g_valid)
//         {
//             // rotation in XY axis
//             float angle = atan(g.y_axis / g.x_axis) - atan(accelerometer.center_g.y_axis / accelerometer.center_g.x_axis);
//             angle *= (180.f / M_PI);

//             if (angle < -90)
//             {
//                 angle = 180 + angle;
//             }

//             if ((angle < STOP_ANGLE) && (angle > -STOP_ANGLE))
//             {
//                 last_angle = angle;

//                 return;
//             }

//             if (angle < 0)
//             {
//                 angle = -angle;
//             }

//             if (angle > max_angle)
//             {
//                 max_angle = angle;
//             }

//             bc_tick_t now = bc_tick_get();

//             if (next_send_tick < now)
//             {
//                 bc_radio_pub_float("a", &max_angle);

//                 max_angle = 0;

//                 next_send_tick = now + 2000;
//             }

//             // if (direction)
//             // {
//             //     if (last_angle > angle)
//             //     {
//             //         bc_radio_pub_float("a", &last_angle);
//             //         direction = false;
//             //     }
//             // }
//             // else
//             // {
//             //     if (last_angle < angle)
//             //     {
//             //         bc_radio_pub_float("a", &last_angle);
//             //         direction = true;
//             //     }
//             // }
//             // last_angle = angle;

// #if LCD
//             bc_system_pll_enable();
//             bc_module_lcd_clear();
//             bc_module_lcd_set_font(&bc_font_ubuntu_28);
//             char str[16];
//             snprintf(str, sizeof(str), "%.2f", angle);
//             bc_module_lcd_draw_string(25, 25, str, true);
//             bc_module_lcd_update();
//             bc_system_pll_disable();
// #endif

//         }
//         else
//         {
//             bc_data_stream_feed(&accelerometer.data_stream.x_axis, &g.x_axis);
//             bc_data_stream_feed(&accelerometer.data_stream.y_axis, &g.y_axis);
//             bc_data_stream_feed(&accelerometer.data_stream.z_axis, &g.z_axis);

//             if (!bc_data_stream_get_median(&accelerometer.data_stream.x_axis, &accelerometer.center_g.x_axis) ||
//                 !bc_data_stream_get_median(&accelerometer.data_stream.y_axis, &accelerometer.center_g.y_axis) ||
//                 !bc_data_stream_get_median(&accelerometer.data_stream.z_axis, &accelerometer.center_g.z_axis))
//             {
//                 return;
//             }

//             accelerometer.center_g_valid = true;
//         }
//    }
// }
