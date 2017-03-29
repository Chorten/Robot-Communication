// File:          NaoEmitter.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
//#include <webots/Robot.hpp>
//#ifdef _WIN32
//#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/HandWave.motion>

#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Keyboard.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/utils/Motion.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Robot.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Emitter.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Motor.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/DifferentialWheels.hpp>
//#else
//#include </Users/Chorten/Desktop/Webots/Projects/include/controller/c/webots/differential_wheels.h>
//#endif


//int wb_emitter_send(WbDeviceTag tag, const void *data, int size);


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int) robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  LED *led = robot->getLED("ledname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  
  //wb_robot_init();
  //DifferentialWheels().setSpeed(100, 100);
  std::string filename = "/Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/HandWave.motion";
  Motion *handwave = new Motion(filename);
  if (! handwave->isValid()) {
    std::cout << "could not load file: " << filename << std::endl;
    delete handwave;
  }
  handwave->setLoop(true);
  handwave->play();
  
  
  //char message[128];
  //sprintf(message, "hello");
  //Emitter(robot->getName()).send(message, sizeof(message) + 1);
  //Motor *move = new Motor(robot->getName());
  //move->setVelocity(100);
  //Keyboard().enable(10);
  //robot-> step(50);
    
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.
    
    // Enter here functions to send actuator commands, like:
    //  led->set(1);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
