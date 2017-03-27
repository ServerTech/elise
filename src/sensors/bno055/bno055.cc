/* Adafruit BNO055 Absolute Orientation Sensor UART Library
 * UART library written for the Raspberry Pi model B revision 3
 * (c) 2017
 *
 * This file is part of 'elise', see: https://github.com/ServerTech/elise
 *
 * Author: Shreyas Vinod
 * Email:  shreyas@shreyasvinod.me
 */

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <assert.h>

#include <stdexcept>

#include "bno055.h"

BNO055::BNO055(char* serial_port) // TODO: rst
:fd_(-1), bno_id_(-1), op_mode_(OPERATION_MODE_NDOF)
{
  fd_ = open(serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ == -1)
    throw std::runtime_error("Port open failure.");
  
  struct termios config;
  tcgetattr(fd_, &config);

  cfsetispeed(&config, B115200);
  cfsetospeed(&config, B115200);

  config.c_cflag |= CLOCAL;
  config.c_cflag |= CREAD;

  // set to 8N1
  config.c_cflag &= ~PARENB;
  config.c_cflag &= ~CSTOPB;
  config.c_cflag &= ~CSIZE;
  config.c_cflag |= CS8;

  config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  config.c_iflag &= ~(IXON | IXOFF | IXANY);
  config.c_oflag &= ~OPOST;

  tcflush(fd_, TCIFLUSH);
  tcsetattr(fd_, TCSANOW, &config);
  fcntl(fd_, F_SETFL, FNDELAY);
}

BNO055::~BNO055()
{
  close(fd_);
}

void BNO055::serialSend(BYTE_T* command, int length, BYTE_T* response,
                        bool ack, int max_attempts)
{
  if (fd_ == -1)
    throw std::runtime_error("Port not open.");

  int attempts = 0;

  while (1)
  {
    tcflush(fd_, TCIFLUSH);
    int tx_length = write(fd_, command, length);
    if (tx_length < 0)
      throw std::runtime_error("Write error.");

    if (!ack) return;

    usleep(RX_WAIT_TIME * 1000);
    int rx_length = read(fd_, response, 2);
    if (rx_length < 0)
      throw std::runtime_error("ACK read error.");

    if (response[0] != 0xEE || response[1] != 0x07) // bus error: 0xEE07
      return;

    attempts++;

    if (attempts >= max_attempts)
      throw std::runtime_error("Write failure due to bus error.");
  }
}

void BNO055::readBytes(BYTE_T address, int length, BYTE_T* response)
{
  if (fd_ == -1)
    throw std::runtime_error("Port not open.");

  BYTE_T command[4];
  command[0] = 0xAA; // start byte
  command[1] = 0x01; // read
  command[2] = address;
  command[3] = length & 0xFF;

  BYTE_T tx_response[2];
  this->serialSend(command, 4, tx_response);
  if (tx_response[0] != 0xBB)
    throw std::runtime_error("Register read error.");

  length = tx_response[1];
  int rx_length = read(fd_, response, length);
  if (rx_length < 0)
    throw std::runtime_error("Read error.");

  assert(rx_length == length);
}

BYTE_T BNO055::readByte(BYTE_T address)
{
  BYTE_T response;
  this->readBytes(address, 1, &response);
  return response;
}

SBYTE_T BNO055::readSignedByte(BYTE_T address)
{
  BYTE_T u_byte = this->readByte(address);
  SBYTE_T s_byte = (u_byte > 127) ? u_byte - 256 : u_byte;
  return s_byte;
}

void BNO055::writeBytes(BYTE_T address, BYTE_T* data, int length, bool ack)
{
  BYTE_T* command = new BYTE_T[4 + length];
  command[0] = 0xAA; // start byte
  command[1] = 0x00; // write
  command[2] = address;
  command[3] = length & 0xFF;

  for (int i = 0; i < length; ++i)
    command[4 + i] = data[i];

  BYTE_T tx_response[2];
  this->serialSend(command, length + 4, tx_response, ack);
  if (ack)
  {
    if (tx_response[0] != 0xEE && tx_response[1] != 0x01)
      throw std::runtime_error("Register write error.");
  }

  delete command;
}

void BNO055::writeByte(BYTE_T address, BYTE_T value, bool ack)
{
  this->writeBytes(address, &value, 1, ack);
}

void BNO055::configMode()
{
  this->setMode(OPERATION_MODE_CONFIG);
}

void BNO055::operationMode()
{
  this->setMode(op_mode_);
}

bool BNO055::begin(BYTE_T op_mode)
{
  op_mode_ = op_mode;

  try { this->writeByte(BNO055_PAGE_ID_ADDR, 0, 0); }
  catch (std::exception& e) {}; // swallow!

  this->configMode(); // set config mode
  this->writeByte(BNO055_PAGE_ID_ADDR, 0); // and page 0

  bno_id_ = this->readByte(BNO055_CHIP_ID_ADDR); // check device ID
  if (bno_id_ != BNO055_ID)
    return 0;

  this->writeByte(BNO055_SYS_TRIGGER_ADDR, 0x20, 0); // reset the device

  usleep(650 * 1000); // wait 650 ms for device
  this->writeByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL); // normal power
  this->writeByte(BNO055_SYS_TRIGGER_ADDR, 0x00); // internal oscillator
  this->operationMode();
  return 1;
}

void BNO055::setMode(BYTE_T op_mode)
{
  this->writeByte(BNO055_OPR_MODE_ADDR, op_mode);
}

void BNO055::getRevision(BYTE_T* revision)
{
  revision[0] = this->readByte(BNO055_ACCEL_REV_ID_ADDR);
  revision[1] = this->readByte(BNO055_MAG_REV_ID_ADDR);
  revision[2] = this->readByte(BNO055_GYRO_REV_ID_ADDR);
  revision[3] = this->readByte(BNO055_BL_REV_ID_ADDR);
  revision[4] = this->readByte(BNO055_SW_REV_ID_LSB_ADDR);
  revision[5] = this->readByte(BNO055_SW_REV_ID_MSB_ADDR);
}

void BNO055::setExternalCrystal(bool external_crystal)
{
  this->configMode();

  if (external_crystal)
    this->writeByte(BNO055_SYS_TRIGGER_ADDR, 0x80);
  else
    this->writeByte(BNO055_SYS_TRIGGER_ADDR, 0x00);

  this->operationMode();
}

void BNO055::readVector(BYTE_T address, double* data, int count)
{
  BYTE_T* byte_data = new BYTE_T[count * 2];
  this->readBytes(address, count * 2, byte_data);

  for (int i = 0; i < count; ++i)
  {
    data[i] = ((byte_data[i*2+1] << 8) | byte_data[i*2]);
    if (data[i] > 32767)
      data[i] -= 65536;
  }

  delete byte_data;
}

void BNO055::readEuler(double* response)
{
  this->readVector(BNO055_EULER_H_LSB_ADDR, response);

  for (int i = 0; i < 3; ++i)
    response[i] /= 16.0;
}

void BNO055::readMagnetometer(double* response)
{
  this->readVector(BNO055_MAG_DATA_X_LSB_ADDR, response);

  for (int i = 0; i < 3; ++i)
    response[i] /= 16.0;
}

void BNO055::readGyroscope(double* response)
{
  this->readVector(BNO055_GYRO_DATA_X_LSB_ADDR, response);

  for (int i = 0; i < 3; ++i)
    response[i] /= 900.0;
}

void BNO055::readAccelerometer(double* response)
{
  this->readVector(BNO055_ACCEL_DATA_X_LSB_ADDR, response);

  for (int i = 0; i < 3; ++i)
    response[i] /= 100.0;
}

void BNO055::readLinearAcceleration(double* response)
{
  this->readVector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, response);

  for (int i = 0; i < 3; ++i)
    response[i] /= 100.0;
}

void BNO055::readGravity(double* response)
{
  this->readVector(BNO055_GRAVITY_DATA_X_LSB_ADDR, response);

  for (int i = 0; i < 3; ++i)
    response[i] /= 100.0;
}

void BNO055::readQuaternion(double* response)
{
  this->readVector(BNO055_QUATERNION_DATA_W_LSB_ADDR, response, 4);

  for (int i = 0; i < 3; ++i)
    response[i] *= (1.0 / (1 << 14));
}

SBYTE_T BNO055::readTemp()
{
  return this->readSignedByte(BNO055_TEMP_ADDR);
}