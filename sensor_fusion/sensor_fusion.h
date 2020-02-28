#ifndef __MPU6050_H
#define __MPU6050_H

#include "mbed.h"

/** TO DO
 * DEFINE THESE REGISTERS. 
 * THE ADDRESS AND CONFIG REGISTERS HAVE BEEN DONE FOR YOU AS AN EXAMPLE.
 *
 * You will need to look at the register map.
 */

#define SDA D4            //the pin used for SDA
#define SCL D5            //the pin used for SCL
#define ADDRESS 0xD1      //the slave address of the IMU
#define INT_STATUS 0x3A   //the interrupt status register
#define PWR_MGMT_1 0x6B   //power management 1
#define CONFIG 0x1A       //the IMU config register
#define GYRO_CONFIG 0x1B  //the gyroscope config register
#define ACCEL_CONFIG 0x1C //the accelerometer config register
#define GYRO_X 0x43       //the register that stores the first bit of the gyroscope x value
#define ACCEL_X 0x3B      //the register that stores the first bit of the accelerometer x value

class MPU6050
{

public:
    /** 
     * Creates an MPU6050 object
     * 
     * sda: pin name of the sda pin
     * scl: pin name of the scl pin
     */
    MPU6050(PinName sda, PinName scl);

    /** 
     * Called before using the MPU6050.
     * 
     * Sets up the I2C by configuring registers 
     * as described in the spec (calls write_reg).
     */
    void start(void);

    /** 
     * Gets the readings from the gyroscope and accelerometer (calls read_reg).
     * 
     * Parameters: pointers to addresses where these readings should be stored.
     *
     * Returns true upon success, and false upon failure.
     */
    bool read_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az);

    /**
     * Reads the interrupt status register (calls read_reg).
     *
     * Returns true if data is ready, false otherwise.
     */
    bool data_ready(void);

private:
    I2C i2c_object; //instance of an i2c class

    /** 
     * Writes 1 byte of data to a specific address
     * 
     * addr: the i2c address of the slave device
     * reg: the register number to write to
     * buf: the data to write
     *
     * Returns true upon success, and false upon failure.
     */
    bool write_reg(int addr, char reg, char buf);

    /** 
     * Reads data from a specific address
     * 
     * addr: the i2c address of the slave device
     * reg: the register numer to read from
     * buf: an array to store the data read
     * length: length of buf (how much to read)
     *
     * Returns true upon success, and false upon failure.
     */
    bool read_reg(int addr, char reg, char *buf, int length);
};

#endif
