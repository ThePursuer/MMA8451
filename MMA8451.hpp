#ifndef MMA8451_DRIVER_H_
#define MMA8451_DRIVER_H_

#include <cstdint>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <unistd.h>

namespace MMA8451 {

// MMA8451 register addresses
// MMA8451 register addresses
enum class MMA8451_Register : uint8_t {
  STATUS = 0x00,
  OUT_X_MSB = 0x01,
  OUT_X_LSB = 0x02,
  OUT_Y_MSB = 0x03,
  OUT_Y_LSB = 0x04,
  OUT_Z_MSB = 0x05,
  OUT_Z_LSB = 0x06,
  F_SETUP = 0x09,
  TRIG_CFG = 0x0A,
  SYSMOD = 0x0B,
  INT_SOURCE = 0x0C,
  WHO_AM_I = 0x0D,
  XYZ_DATA_CFG = 0x0E,
  HP_FILTER_CUTOFF = 0x0F,
  PL_STATUS = 0x10,
  PL_CFG = 0x11,
  PL_COUNT = 0x12,
  PL_BF_ZCOMP = 0x13,
  PL_THS_REG = 0x14,
  FF_MT_CFG = 0x15,
  FF_MT_SRC = 0x16,
  FF_MT_THS = 0x17,
  FF_MT_COUNT = 0x18,
  TRANSIENT_CFG = 0x1D,
  TRANSIENT_SRC = 0x1E,
  TRANSIENT_THS = 0x1F,
  TRANSIENT_COUNT = 0x20,
  PULSE_CFG = 0x21,
  PULSE_SRC = 0x22,
  PULSE_THSX = 0x23,
  PULSE_THSY = 0x24,
  PULSE_THSZ = 0x25,
  PULSE_TMLT = 0x26,
  PULSE_LTCY = 0x27,
  PULSE_WIND = 0x28,
  ASLP_COUNT = 0x29,
  CTRL_REG1 = 0x2A,
  CTRL_REG2 = 0x2B,
  CTRL_REG3 = 0x2C,
  CTRL_REG4 = 0x2D,
  CTRL_REG5 = 0x2E,
  OFF_X = 0x2F,
  OFF_Y = 0x30,
  OFF_Z = 0x31
};

struct AccelerometerData {
  float x;
  float y;
  float z;
};

class MMA8451_Driver {
 public:
  explicit MMA8451_Driver(int i2c_fd) : i2c_fd_(i2c_fd) {}

  bool Init();
  AccelerometerData ReadAccelerometer();
  bool SelfTest();
 private:
  int i2c_fd_;

  uint8_t ReadRegister(MMA8451_Register reg);
  void WriteRegister(MMA8451_Register reg, uint8_t value);
};

}  // namespace my_namespace

#endif  // MMA8451_DRIVER_H_