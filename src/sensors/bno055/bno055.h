/* Adafruit BNO055 Absolute Orientation Sensor UART Library
 * UART library written for the Raspberry Pi model B revision 3
 * (c) 2017
 *
 * This file is part of 'elise', see: https://github.com/ServerTech/elise
 *
 * Author: Shreyas Vinod
 * Email:  shreyas@shreyasvinod.me
 */

#ifndef BNO055_H_
#define BNO055_H_

#include <cstdint>

using BYTE_T = uint8_t;
using SBYTE_T = int8_t;

const int RX_WAIT_TIME = 10; // milliseconds

// I2C addresses
const BYTE_T BNO055_ADDRESS_A                     = 0x28;
const BYTE_T BNO055_ADDRESS_B                     = 0x29;
const BYTE_T BNO055_ID                            = 0xA0;

// Page ID register definition
const BYTE_T BNO055_PAGE_ID_ADDR                  = 0x07;

// Page 0 register definition start
const BYTE_T BNO055_CHIP_ID_ADDR                  = 0x00;
const BYTE_T BNO055_ACCEL_REV_ID_ADDR             = 0x01;
const BYTE_T BNO055_MAG_REV_ID_ADDR               = 0x02;
const BYTE_T BNO055_GYRO_REV_ID_ADDR              = 0x03;
const BYTE_T BNO055_SW_REV_ID_LSB_ADDR            = 0x04;
const BYTE_T BNO055_SW_REV_ID_MSB_ADDR            = 0x05;
const BYTE_T BNO055_BL_REV_ID_ADDR                = 0x06;

// Accel data register
const BYTE_T BNO055_ACCEL_DATA_X_LSB_ADDR         = 0x08;
const BYTE_T BNO055_ACCEL_DATA_X_MSB_ADDR         = 0x09;
const BYTE_T BNO055_ACCEL_DATA_Y_LSB_ADDR         = 0x0A;
const BYTE_T BNO055_ACCEL_DATA_Y_MSB_ADDR         = 0x0B;
const BYTE_T BNO055_ACCEL_DATA_Z_LSB_ADDR         = 0x0C;
const BYTE_T BNO055_ACCEL_DATA_Z_MSB_ADDR         = 0x0D;

// Mag data register
const BYTE_T BNO055_MAG_DATA_X_LSB_ADDR           = 0x0E;
const BYTE_T BNO055_MAG_DATA_X_MSB_ADDR           = 0x0F;
const BYTE_T BNO055_MAG_DATA_Y_LSB_ADDR           = 0x10;
const BYTE_T BNO055_MAG_DATA_Y_MSB_ADDR           = 0x11;
const BYTE_T BNO055_MAG_DATA_Z_LSB_ADDR           = 0x12;
const BYTE_T BNO055_MAG_DATA_Z_MSB_ADDR           = 0x13;

// Gyro data registers
const BYTE_T BNO055_GYRO_DATA_X_LSB_ADDR          = 0x14;
const BYTE_T BNO055_GYRO_DATA_X_MSB_ADDR          = 0x15;
const BYTE_T BNO055_GYRO_DATA_Y_LSB_ADDR          = 0x16;
const BYTE_T BNO055_GYRO_DATA_Y_MSB_ADDR          = 0x17;
const BYTE_T BNO055_GYRO_DATA_Z_LSB_ADDR          = 0x18;
const BYTE_T BNO055_GYRO_DATA_Z_MSB_ADDR          = 0x19;

// Euler data registers
const BYTE_T BNO055_EULER_H_LSB_ADDR              = 0x1A;
const BYTE_T BNO055_EULER_H_MSB_ADDR              = 0x1B;
const BYTE_T BNO055_EULER_R_LSB_ADDR              = 0x1C;
const BYTE_T BNO055_EULER_R_MSB_ADDR              = 0x1D;
const BYTE_T BNO055_EULER_P_LSB_ADDR              = 0x1E;
const BYTE_T BNO055_EULER_P_MSB_ADDR              = 0x1F;

// Quaternion data registers
const BYTE_T BNO055_QUATERNION_DATA_W_LSB_ADDR    = 0x20;
const BYTE_T BNO055_QUATERNION_DATA_W_MSB_ADDR    = 0x21;
const BYTE_T BNO055_QUATERNION_DATA_X_LSB_ADDR    = 0x22;
const BYTE_T BNO055_QUATERNION_DATA_X_MSB_ADDR    = 0x23;
const BYTE_T BNO055_QUATERNION_DATA_Y_LSB_ADDR    = 0x24;
const BYTE_T BNO055_QUATERNION_DATA_Y_MSB_ADDR    = 0x25;
const BYTE_T BNO055_QUATERNION_DATA_Z_LSB_ADDR    = 0x26;
const BYTE_T BNO055_QUATERNION_DATA_Z_MSB_ADDR    = 0x27;

// Linear acceleration data registers
const BYTE_T BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0x28;
const BYTE_T BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0x29;
const BYTE_T BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0x2A;
const BYTE_T BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0x2B;
const BYTE_T BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0x2C;
const BYTE_T BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0x2D;

// Gravity data registers
const BYTE_T BNO055_GRAVITY_DATA_X_LSB_ADDR       = 0x2E;
const BYTE_T BNO055_GRAVITY_DATA_X_MSB_ADDR       = 0x2F;
const BYTE_T BNO055_GRAVITY_DATA_Y_LSB_ADDR       = 0x30;
const BYTE_T BNO055_GRAVITY_DATA_Y_MSB_ADDR       = 0x31;
const BYTE_T BNO055_GRAVITY_DATA_Z_LSB_ADDR       = 0x32;
const BYTE_T BNO055_GRAVITY_DATA_Z_MSB_ADDR       = 0x33;

// Temperature data register
const BYTE_T BNO055_TEMP_ADDR                     = 0x34;

// Status registers
const BYTE_T BNO055_CALIB_STAT_ADDR               = 0x35;
const BYTE_T BNO055_SELFTEST_RESULT_ADDR          = 0x36;
const BYTE_T BNO055_INTR_STAT_ADDR                = 0x37;

const BYTE_T BNO055_SYS_CLK_STAT_ADDR             = 0x38;
const BYTE_T BNO055_SYS_STAT_ADDR                 = 0x39;
const BYTE_T BNO055_SYS_ERR_ADDR                  = 0x3A;

// Unit selection register
const BYTE_T BNO055_UNIT_SEL_ADDR                 = 0x3B;
const BYTE_T BNO055_DATA_SELECT_ADDR              = 0x3C;

// Mode registers
const BYTE_T BNO055_OPR_MODE_ADDR                 = 0x3D;
const BYTE_T BNO055_PWR_MODE_ADDR                 = 0x3E;

const BYTE_T BNO055_SYS_TRIGGER_ADDR              = 0x3F;
const BYTE_T BNO055_TEMP_SOURCE_ADDR              = 0x40;

// Axis remap registers
const BYTE_T BNO055_AXIS_MAP_CONFIG_ADDR          = 0x41;
const BYTE_T BNO055_AXIS_MAP_SIGN_ADDR            = 0x42;

// Axis remap values
const BYTE_T AXIS_REMAP_X                         = 0x00;
const BYTE_T AXIS_REMAP_Y                         = 0x01;
const BYTE_T AXIS_REMAP_Z                         = 0x02;
const BYTE_T AXIS_REMAP_POSITIVE                  = 0x00;
const BYTE_T AXIS_REMAP_NEGATIVE                  = 0x01;

// SIC registers
const BYTE_T BNO055_SIC_MATRIX_0_LSB_ADDR         = 0x43;
const BYTE_T BNO055_SIC_MATRIX_0_MSB_ADDR         = 0x44;
const BYTE_T BNO055_SIC_MATRIX_1_LSB_ADDR         = 0x45;
const BYTE_T BNO055_SIC_MATRIX_1_MSB_ADDR         = 0x46;
const BYTE_T BNO055_SIC_MATRIX_2_LSB_ADDR         = 0x47;
const BYTE_T BNO055_SIC_MATRIX_2_MSB_ADDR         = 0x48;
const BYTE_T BNO055_SIC_MATRIX_3_LSB_ADDR         = 0x49;
const BYTE_T BNO055_SIC_MATRIX_3_MSB_ADDR         = 0x4A;
const BYTE_T BNO055_SIC_MATRIX_4_LSB_ADDR         = 0x4B;
const BYTE_T BNO055_SIC_MATRIX_4_MSB_ADDR         = 0x4C;
const BYTE_T BNO055_SIC_MATRIX_5_LSB_ADDR         = 0x4D;
const BYTE_T BNO055_SIC_MATRIX_5_MSB_ADDR         = 0x4E;
const BYTE_T BNO055_SIC_MATRIX_6_LSB_ADDR         = 0x4F;
const BYTE_T BNO055_SIC_MATRIX_6_MSB_ADDR         = 0x50;
const BYTE_T BNO055_SIC_MATRIX_7_LSB_ADDR         = 0x51;
const BYTE_T BNO055_SIC_MATRIX_7_MSB_ADDR         = 0x52;
const BYTE_T BNO055_SIC_MATRIX_8_LSB_ADDR         = 0x53;
const BYTE_T BNO055_SIC_MATRIX_8_MSB_ADDR         = 0x54;

// Accelerometer Offset registers
const BYTE_T ACCEL_OFFSET_X_LSB_ADDR              = 0x55;
const BYTE_T ACCEL_OFFSET_X_MSB_ADDR              = 0x56;
const BYTE_T ACCEL_OFFSET_Y_LSB_ADDR              = 0x57;
const BYTE_T ACCEL_OFFSET_Y_MSB_ADDR              = 0x58;
const BYTE_T ACCEL_OFFSET_Z_LSB_ADDR              = 0x59;
const BYTE_T ACCEL_OFFSET_Z_MSB_ADDR              = 0x5A;

// Magnetometer Offset registers
const BYTE_T MAG_OFFSET_X_LSB_ADDR                = 0x5B;
const BYTE_T MAG_OFFSET_X_MSB_ADDR                = 0x5C;
const BYTE_T MAG_OFFSET_Y_LSB_ADDR                = 0x5D;
const BYTE_T MAG_OFFSET_Y_MSB_ADDR                = 0x5E;
const BYTE_T MAG_OFFSET_Z_LSB_ADDR                = 0x5F;
const BYTE_T MAG_OFFSET_Z_MSB_ADDR                = 0x60;

// Gyroscope Offset registers
const BYTE_T GYRO_OFFSET_X_LSB_ADDR               = 0x61;
const BYTE_T GYRO_OFFSET_X_MSB_ADDR               = 0x62;
const BYTE_T GYRO_OFFSET_Y_LSB_ADDR               = 0x63;
const BYTE_T GYRO_OFFSET_Y_MSB_ADDR               = 0x64;
const BYTE_T GYRO_OFFSET_Z_LSB_ADDR               = 0x65;
const BYTE_T GYRO_OFFSET_Z_MSB_ADDR               = 0x66;

// Radius registers
const BYTE_T ACCEL_RADIUS_LSB_ADDR                = 0x67;
const BYTE_T ACCEL_RADIUS_MSB_ADDR                = 0x68;
const BYTE_T MAG_RADIUS_LSB_ADDR                  = 0x69;
const BYTE_T MAG_RADIUS_MSB_ADDR                  = 0x6A;

// Power modes
const BYTE_T POWER_MODE_NORMAL                    = 0x00;
const BYTE_T POWER_MODE_LOWPOWER                  = 0x01;
const BYTE_T POWER_MODE_SUSPEND                   = 0x02;

// Operation mode settings
const BYTE_T OPERATION_MODE_CONFIG                = 0x00;
const BYTE_T OPERATION_MODE_ACCONLY               = 0x01;
const BYTE_T OPERATION_MODE_MAGONLY               = 0x02;
const BYTE_T OPERATION_MODE_GYRONLY               = 0x03;
const BYTE_T OPERATION_MODE_ACCMAG                = 0x04;
const BYTE_T OPERATION_MODE_ACCGYRO               = 0x05;
const BYTE_T OPERATION_MODE_MAGGYRO               = 0x06;
const BYTE_T OPERATION_MODE_AMG                   = 0x07;
const BYTE_T OPERATION_MODE_IMUPLUS               = 0x08;
const BYTE_T OPERATION_MODE_COMPASS               = 0x09;
const BYTE_T OPERATION_MODE_M4G                   = 0x0A;
const BYTE_T OPERATION_MODE_NDOF_FMC_OFF          = 0x0B;
const BYTE_T OPERATION_MODE_NDOF                  = 0x0C;

class BNO055
{

 public:

  BNO055(char* serial_port);
  ~BNO055();
  bool begin(BYTE_T op_mode = OPERATION_MODE_NDOF);
  void getRevision(BYTE_T* revision);
  void setExternalCrystal(bool external_crystal);
  void readVector(BYTE_T address, double* data, int count = 3);
  void readEuler(double* response);
  void readMagnetometer(double* response);
  void readGyroscope(double* response);
  void readAccelerometer(double* response);
  void readLinearAcceleration(double* response);
  void readGravity(double* response);
  void readQuaternion(double* response);
  SBYTE_T readTemp();

 private:

  int fd_;
  BYTE_T bno_id_;
  BYTE_T op_mode_;

  void serialSend(BYTE_T* command, int length, BYTE_T* response, bool ack = 1,
                  int max_attempts = 5);
  void readBytes(BYTE_T address, int length, BYTE_T* response);
  BYTE_T readByte(BYTE_T address);
  SBYTE_T readSignedByte(BYTE_T address);
  void writeBytes(BYTE_T address, BYTE_T* data,int length, bool ack = 1);
  void writeByte(BYTE_T address, BYTE_T value, bool ack = 1);
  void configMode();
  void operationMode();
  void setMode(BYTE_T op_mode);

};

#endif // BNO055_H_