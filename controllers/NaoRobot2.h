#include <unistd.h>
#include <cstring>
#include <string>
#include <iostream>

class NaoRobot2
{
  public:
    std::string name;
    long time; //time in microseconds
    int role;
    double x;
    double y;
    double z;
    double velocityX;
    double velocityY;
    double velocityZ;
    
    NaoRobot2(std::string name)
    {
      this->name = name;
      role = -1;
    }
};