// File:          BallController.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
//#include <webots/Robot.hpp>
//#ifdef _WIN32
//#include <webots/HandWave.motion>

#include <webots/Keyboard.hpp>
#include <webots/utils/Motion.hpp>
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <../Data.h>

//#else
//#include </home/viruszer0/Desktop/Webots/include/controller/c/webots/differential_wheels.h>
//#endif

using namespace webots;
using namespace std;
using namespace chrono;

enum Channel
{
  channelSupervisor = 1,
  channelGeneral = 2
};

enum Role
{
  roleUndefined= 0,
  roleAttacker = 1,
  roleDefender = 2,
  roleGoale = 3,
  roleNone = 4
};

size_t MessageID = 1;

class Ball : public Robot
{
  public:
    static const size_t nameSize = 4;
    Ball(char* name);
    void run();
  private:
    int timeStep;
    char* name;
    Emitter* emitter;
    GPS* gps;
    Gyro* gyro;
    long getTime();
};

Ball::Ball(char* name)
{
  // defaults:
    // timeStep (line below): 32
    // WorldInfo, basicTimeStep: 16
    // WOrldInfo, FPS: 60
  timeStep = 64;
  // 32,16,60  - ping ~190 (double for second messages)
  // 8,16,60   - ping ~95
  // 8,16,30   - ping ~95
  // 8,4,30    - ping ~110
  // 8,64,30   - ping ~95
  // 4,16,60   - ping ~90
  // 1,16,60   - ping ~85
  // 16,16,60  - ping ~95
  this->name = name;
  gps = new GPS("gps");
  gyro = new Gyro("gyro");
  gps->enable(timeStep);
  gyro->enable(timeStep);
  emitter = getEmitter("emitter");
  emitter->setChannel(channelGeneral);
}

void Ball::run()
{
  string sName(name);
  int counter = 0;
  size_t MessageID = 1;
  long now;
  
  while(step(timeStep) != -1)
  {
    counter++;
    if (counter == 20)
    {
      cout << endl;
      counter = 0;
    }
  
    const double* loc = gps->getValues();
    const double* speed = gyro->getValues();
    now = getTime();
    
    Data dataSending(MessageID, name, now, roleUndefined, loc[0], loc[1], loc[2], speed[0], speed[1], speed[2]);
    if (counter == 0)
      cout << sName << " sending!!!:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName() << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.z << " " << dataSending.velocityX << " " << dataSending.velocityY << " " << dataSending.velocityZ << endl;

    emitter->send(&dataSending, sizeof(dataSending));
    MessageID++;
  }
}

long Ball::getTime()
{
  //std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  auto nowC = high_resolution_clock::now();
  long now = duration_cast<milliseconds>(nowC.time_since_epoch()).count();
  return now;
  //timeval tv1;
  //gettimeofday(&tv1, 0);
  //time_t now_sec = tv1.tv_sec;
  //suseconds_t now_usec = tv1.tv_usec;
  //return now_sec * 1000000 + now_usec;
}


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  char* name = argv[1];
  // create the Robot instance.
  Ball *ball = new Ball(name);
  ball->run();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  LED *led = robot->getLED("ledname");
  
  //Motor *move = new Motor(robot->getName());
  //move->setVelocity(100);
  //Keyboard().enable(10);
  //robot-> step(50);
  
  // Enter here exit cleanup code.

  delete ball;
  return 0;
}
