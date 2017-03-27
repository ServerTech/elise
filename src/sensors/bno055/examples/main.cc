#include <unistd.h> // usleep()

#include <iostream>
#include <stdexcept>

#include "bno055.h"

int main()
{
  // Adafruit BNO055 9-axis absolute orientation sensor
  // Documentation:  http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

  BNO055 bno((char*) "/dev/ttyS0");

  // The sensor can be initialised in several different modes by passing
  // the correct mode. Modes are defined in bno055.h and in the documentation.

  if (!bno.begin())
    throw std::runtime_error("wtf happened?");

  while (1)
  {
    double euler[3];
    bno.readEuler(euler);

    std::cout << "heading: " << euler[0] << " roll: "
              << euler[1] << " pitch: " << euler[2] << std::endl;

    /*
    Other things: (check bno055.h)

    void readMagnetometer(double* response); // length 3
    void readGyroscope(double* response); // length 3
    void readAccelerometer(double* response); // length 3
    void readLinearAcceleration(double* response); // length 3
    void readGravity(double* response); // length 3
    void readQuaternion(double* response); // length 4
    SBYTE_T readTemp();
    */

    usleep(1000 * 1000); // sleep for one second
  }
}