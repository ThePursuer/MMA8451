#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

#include "MMA8451.hpp"

volatile bool g_quit = false;

void signal_handler(int signum) {
  g_quit = true;
}

void usage(const char* progname) {
  printf("Usage: %s [-f i2c_dev_file]\n", progname);
}

int main(int argc, char** argv) {
  const char* i2c_dev_file = "/dev/i2c-1";

  // Parse command line options
  int opt;
  while ((opt = getopt(argc, argv, "f:")) != -1) {
    switch (opt) {
      case 'f':
        i2c_dev_file = optarg;
        break;
      default:
        usage(argv[0]);
        return EXIT_FAILURE;
    }
  }
  // Initialize MMA8451 driver
  int i2c_fd = open(i2c_dev_file, O_RDWR);
  if (i2c_fd < 0) {
    printf("Failed to open I2C device file %s\n", i2c_dev_file);
    return EXIT_FAILURE;
  }
  MMA8451::MMA8451_Driver mma8451(i2c_fd);
  mma8451.SelfTest();
  if (!mma8451.Init()) {
    printf("Failed to initialize MMA8451\n");
    return EXIT_FAILURE;
  }
  std::cout << "Here" << std::endl;

  // Register signal handler for SIGINT
  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = signal_handler;
  sigaction(SIGINT, &action, NULL);

  // Read accelerometer data until Ctrl+C is pressed
  while (!g_quit) {
    MMA8451::AccelerometerData data = mma8451.ReadAccelerometer();
    printf("Accelerometer data: x=%.3f G, y=%.3f G, z=%.3f G\n", data.x, data.y, data.z);
    usleep(100000);
  }

  // Close I2C device file
  close(i2c_fd);

  return EXIT_SUCCESS;
}