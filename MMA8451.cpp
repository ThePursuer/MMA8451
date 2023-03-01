#include "MMA8451.hpp"

#include <cstring>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

namespace MMA8451 {

bool MMA8451_Driver::Init() {
  // Set the device to standby mode to configure the registers
  WriteRegister(MMA8451_Register::CTRL_REG1, 0x00);

  // Configure the HPF and LPF filters
  uint8_t hpf_cfg = 0x10; // Enable the HPF with a cutoff frequency of 0.8Hz
  uint8_t lpf_cfg = 0x00; // Disable the LPF
  WriteRegister(MMA8451_Register::HP_FILTER_CUTOFF, hpf_cfg);
  WriteRegister(MMA8451_Register::CTRL_REG2, lpf_cfg);

  // Configure the accelerometer for high resolution mode and +/- 2g range
  uint8_t ctrl_reg1_val = 0x39; // High resolution mode, ODR=800Hz, Active mode
  WriteRegister(MMA8451_Register::CTRL_REG1, ctrl_reg1_val);

  // Wait for the device to become active
  while (!(ReadRegister(MMA8451_Register::CTRL_REG1) & 0x01)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return true;
}

AccelerometerData MMA8451_Driver::ReadAccelerometer() {
  AccelerometerData data;

  // Read raw accelerometer values from the MMA8451
  uint8_t buf[6];
  buf[0] = ReadRegister(MMA8451_Register::OUT_X_MSB);
  buf[1] = ReadRegister(MMA8451_Register::OUT_X_LSB);
  buf[2] = ReadRegister(MMA8451_Register::OUT_Y_MSB);
  buf[3] = ReadRegister(MMA8451_Register::OUT_Y_LSB);
  buf[4] = ReadRegister(MMA8451_Register::OUT_Z_MSB);
  buf[5] = ReadRegister(MMA8451_Register::OUT_Z_LSB);

  // Convert raw accelerometer values to G's
  int16_t raw_x = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  int16_t raw_y = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  int16_t raw_z = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  data.x = static_cast<float>(raw_x) / 4096.0f;
  data.y = static_cast<float>(raw_y) / 4096.0f;
  data.z = static_cast<float>(raw_z) / 4096.0f;

  return data;
}

uint8_t MMA8451_Driver::ReadRegister(MMA8451_Register reg) {
  uint8_t value = 0;
  uint8_t reg_addr = static_cast<uint8_t>(reg);
  if (ioctl(i2c_fd_, I2C_SLAVE, 0x1D) < 0) {
    throw std::runtime_error("Failed to set I2C slave address");
  }
  struct i2c_msg msg[2];
  msg[0].addr = 0x1D;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = &reg_addr;
  msg[1].addr = 0x1D;
  msg[1].flags = I2C_M_RD;
  msg[1].len = 1;
  msg[1].buf = &value;
  struct i2c_rdwr_ioctl_data data;
  data.msgs = msg;
  data.nmsgs = 2;
  if (ioctl(i2c_fd_, I2C_RDWR, &data) < 0) {
    throw std::runtime_error("Failed to read I2C register value");
  }
  return value;
}

void MMA8451_Driver::WriteRegister(MMA8451_Register reg, uint8_t value) {
  uint8_t buf[2] = {static_cast<uint8_t>(reg), value};
  if (ioctl(i2c_fd_, I2C_SLAVE, 0x1D) < 0) {
    throw std::runtime_error("Failed to set I2C slave address");
  }
  struct i2c_msg msgs[1];
  msgs[0].addr = 0x1D;
  msgs[0].flags = 0;
  msgs[0].len = sizeof(buf);
  msgs[0].buf = buf;
  struct i2c_rdwr_ioctl_data data;
  data.msgs = msgs;
  data.nmsgs = 1;
  if (ioctl(i2c_fd_, I2C_RDWR, &data) < 0) {
    throw std::runtime_error("Failed to write I2C register value");
  }
}

bool MMA8451_Driver::SelfTest() {
  // Save current values of CTRL_REG1 and XYZ_DATA_CFG registers
  uint8_t ctrl_reg1 = ReadRegister(MMA8451_Register::CTRL_REG1);
  uint8_t xyz_data_cfg = ReadRegister(MMA8451_Register::XYZ_DATA_CFG);

  // Enable self-test mode and set full-scale range to +/- 8g
  WriteRegister(MMA8451_Register::XYZ_DATA_CFG, 0x10);
  WriteRegister(MMA8451_Register::CTRL_REG1, 0x3A);

  // Wait for self-test to complete
  usleep(100000);  // Wait 100 ms

  // Read accelerometer values
  AccelerometerData data = ReadAccelerometer();

  // Disable self-test mode and restore original register values
  WriteRegister(MMA8451_Register::CTRL_REG1, ctrl_reg1);
  WriteRegister(MMA8451_Register::XYZ_DATA_CFG, xyz_data_cfg);

  // Compare the measured acceleration values to the expected values
  const float kSelfTestThreshold = 0.2f;  // +/- 20% tolerance
  if (fabs(data.x - 1.0f) > kSelfTestThreshold) {
    printf("Self-test failed: x-axis = %f (expected 1.0)\n", data.x);
    return false;
  }
  if (fabs(data.y + 1.0f) > kSelfTestThreshold) {
    printf("Self-test failed: y-axis = %f (expected -1.0)\n", data.y);
    return false;
  }
  if (fabs(data.z - 0.0f) > kSelfTestThreshold) {
    printf("Self-test failed: z-axis = %f (expected 0.0)\n", data.z);
    return false;
  }

  printf("Self-test passed\n");
  return true;
}

}  // namespace MMA8451