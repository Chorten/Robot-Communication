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
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>

//#else
//#include </home/viruszer0/Desktop/Webots/include/controller/c/webots/differential_wheels.h>
//#endif

using namespace webots;
using namespace std;


class Ball : public Robot
{
  public:
    NaoRobot(char* name);
    void run();
  private:
    int timeStep;
    char* name;
    Receiver* receiver;
    Emitter* emitter;
    DistanceSensor *distanceSensor1;
    DistanceSensor *distanceSensor2;
    PositionSensor *positionSensor1;
    PositionSensor *positionSensor2;
    PositionSensor *positionSensor3;
    PositionSensor *positionSensor4;
};

NaoRobot::NaoRobot(char* name)
{
  timeStep = 32;
  this->name = name;
  emitter = getEmitter("emitter");
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  distanceSensor1 = getDistanceSensor("Sonar/Right");
  distanceSensor2 = getDistanceSensor("Sonar/Left");
  positionSensor1 = getPositionSensor("HeadYawS");
  positionSensor2 = getPositionSensor("HeadPitchS");
  positionSensor3 = getPositionSensor("RShoulderPitchS");
  positionSensor4 = getPositionSensor("RShoulderRollS");
  distanceSensor1->enable(timeStep);
  distanceSensor2->enable(timeStep);
}

void NaoRobot::run()
{

  std::string filename = "/home/viruszer0/Desktop/Repo/Senior-Design-Project/controllers/motions/HandWave.motion";
  Motion *handwave = new Motion(filename);
  if (! handwave->isValid())
  {
    std::cout << "could not load file: " << filename << std::endl;
    delete handwave;
  }
  handwave->setLoop(true);
  handwave->play();
  handwave->stop();
  std::string filename2 = "/home/viruszer0/Desktop/Repo/Senior-Design-Project/controllers/motions/Forwards.motion";
  Motion *walk = new Motion(filename2);
  if (! walk->isValid())
  {
    std::cout << "could not load file: " << filename2 << std::endl;
    delete walk;
  }
  walk->setLoop(true);
  walk->play();
  
  string sName(name);
  string stringMessageS = "Hello, my name is " + sName;
  const char* messageS = stringMessageS.c_str();
  
  emitter->send(messageS, 22);
  
  while(step(timeStep) != -1)
  {
    double distanceVal1 = distanceSensor1->getValue();
    double distanceVal2 = distanceSensor2->getValue();
    
    if (receiver->getQueueLength()>0){
      string messageR((const char*)receiver->getData());
      receiver->nextPacket();

      cout << name << " received: " << messageR << endl;
    }
  }   
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
  NaoRobot *robot = new NaoRobot(name);
  robot->run();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  LED *led = robot->getLED("ledname");
  
  //Motor *move = new Motor(robot->getName());
  //move->setVelocity(100);
  //Keyboard().enable(10);
  //robot-> step(50);
  
  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
