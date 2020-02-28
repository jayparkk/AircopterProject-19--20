#include "mbed.h"
#include "sensor_fusion.h"
#include "quaternion.h"
#include "millis.h"

MPU6050 mpu_object(SDA, SCL);

Serial pc(USBTX, USBRX, 115200);

const float GYRO_TYP = 16.4;
struct vector unfiltered = {0, 0, 1};
struct vector c_filter = {0, 0, 1};

int main()
{
    /* call init*/
    millis_begin();
    float time_passed = millis();
    mpu_object.start();

    /* collect bias */
    float accel_x_bias = 0;
    float accel_y_bias = 0;
    float accel_z_bias = 0;
    float gyro_x_bias = 0;
    float gyro_y_bias = 0;
    float gyro_z_bias = 0;
    for (int i = 0; i < 20; i++)
    {
        mpu_object.read_raw(&gyro_x_bias, &gyro_y_bias, &gyro_z_bias, &accel_x_bias, &accel_y_bias, &accel_z_bias);
        accel_x_bias += accel_x_bias;
        accel_y_bias += accel_y_bias;
        accel_z_bias += accel_z_bias;
        gyro_x_bias += gyro_x_bias;
        gyro_y_bias += gyro_y_bias;
        gyro_z_bias += gyro_z_bias;
    }
    accel_x_bias /= float(20.0f);
    accel_y_bias /= float(20.0f);
    accel_z_bias /= float(20.0f);
    gyro_x_bias /= float(20.0f);
    gyro_y_bias /= float(20.0f);
    gyro_z_bias /= float(20.0f);

    //    printf("%f %f %f %f %f %f\r\n" , accel_x_bias, accel_y_bias, accel_z_bias,
    //    gyro_x_bias, gyro_y_bias, gyro_z_bias);

    float fuckthis = 3.0;
    printf("%f", fuckthis);

    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f, gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;

    /* collect */
    while (1)
    {
        if (!mpu_object.data_ready())
        {
            continue;
        }
        /* some millis fuckery after collecting bias */
        unsigned long t = millis();
        float time_diff = (t - time_passed) / 1000.0;
        time_passed = t;

        //    mpu_object.read_raw(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        mpu_object.read_raw(&gyro_x, &gyro_y, &gyro_z, &accel_x, &accel_y, &accel_z);
        //    printf("%f %f %f %f %f %f\r\n" , accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        accel_x = accel_x - accel_x_bias;
        accel_y = accel_y - accel_y_bias;
        accel_z = accel_z - accel_z_bias + 2000;
        gyro_x = gyro_x - gyro_x_bias;
        gyro_y = gyro_y - gyro_y_bias;
        gyro_z = gyro_z - gyro_z_bias;

        vector accel = {
            accel_x,
            accel_y,
            accel_z};

        vector gyro = {
            gyro_x,
            gyro_y,
            gyro_z};

        vector_normalize(&accel, &accel);
        vector_normalize(&gyro, &gyro);

        struct vector u;
        float mag = vector_normalize(&gyro, &u);
        /* one of the two */
        float angle = mag * time_diff * 3.14159265359 / 180 / GYRO_TYP;
        struct quaternion fuck_this_lab;
        quaternion_create(&u, -angle, &fuck_this_lab);
        quaternion_rotate(&unfiltered, &fuck_this_lab, &unfiltered);

        //    const float alpha = 0.1;
        //    quaternion_rotate(&c_filter, &fuck_this_lab, &c_filter);
        //    vector_multiply(&c_filter, alpha, &c_filter);
        //    struct vector test;
        //    vector_multiply(&accel, 1-alpha, &test);
        //    vector_add(&c_filter, &test, &c_filter);
        //    vector_normalize(&c_filter, &c_filter);

        //    printf("%f %f %f %f %f %f %f %f %f\r\n" , accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, unfiltered.x, unfiltered.y, unfiltered.z);

        wait(0.1); // 100 ms
    }
}