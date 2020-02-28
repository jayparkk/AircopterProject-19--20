#include "mbed.h"
#include "sensor_fusion.h"

MPU6050::MPU6050(PinName sda, PinName scl) : i2c_object(sda, scl)
{
    i2c_object.frequency(400000);
}

void MPU6050::start(void)
{
    write_reg(ADDRESS, PWR_MGMT_1, 0x00);
    write_reg(ADDRESS, GYRO_CONFIG, 0x03 << 3);
    write_reg(ADDRESS, ACCEL_CONFIG, 0x00);
    write_reg(ADDRESS, CONFIG, 0x00);
}

bool MPU6050::read_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az)
{
    char raw_gyro[6];
    char raw_accel[6];
    if (read_reg(ADDRESS, GYRO_X, raw_gyro, 6))
    {
        read_reg(ADDRESS, ACCEL_X, raw_accel, 6);
        *gx = float(short(raw_gyro[0] << 8 | raw_gyro[1]));
        *gy = float(short(raw_gyro[2] << 8 | raw_gyro[3]));
        *gz = float(short(raw_gyro[4] << 8 | raw_gyro[5]));
        *ax = float(short(raw_accel[0] << 8 | raw_accel[1]));
        *ay = float(short(raw_accel[2] << 8 | raw_accel[3]));
        *az = float(short(raw_accel[4] << 8 | raw_accel[5]));
        return true;
    }
    return false;
}

bool MPU6050::data_ready(void)
{
    char int_status;
    read_reg(ADDRESS, INT_STATUS, &int_status, 1);
    return int_status & 1;
}

bool MPU6050::write_reg(int i2c, char reg, char buf)
{
    char data[2] = {reg, buf};
    return MPU6050::i2c_object.write(i2c, data, 2) == 0;
}

bool MPU6050::read_reg(int addr, char reg, char *buf, int length)
{
    return i2c_object.write(addr, &reg, 1, true) == 0 &&
           i2c_object.read(addr, buf, length) == 0;
}
