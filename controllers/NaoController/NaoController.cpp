// File:          NaoController.cpp
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


/* TODO:  - Wrap emitter & receiver into functions
          - map sensors to real values
          - should probably recieve and then send (right now other way around)
          - sending on different channels dont seem to work for some reason...
*/


#include <webots/Keyboard.hpp>
#include <webots/utils/Motion.hpp>
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
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

class NaoRobot : public Robot
{
  public:
    static const size_t nameSize = 4;
    NaoRobot(char* name);
    void run();
  private:
    int timeStep;
    char* name;
    Receiver* receiver;
    Emitter* emitter;
    DistanceSensor *distanceSensorSonarRight;
    DistanceSensor *distanceSensorSonarLeft;
    PositionSensor *positionSensorHeadYawS;
    PositionSensor *positionSensorHeadPitchS;
    PositionSensor *positionSensor3;
    PositionSensor *positionSensor4;
    GPS* gps;
    long getTime();
};

NaoRobot::NaoRobot(char* name)
{
  // defaults:
    // timeStep (line below): 32
    // WorldInfo, basicTimeStep: 16
    // WOrldInfo, FPS: 60
  timeStep = 16;
  // 32,16,60  - ping ~190 (double for second messages)
  // 8,16,60   - ping ~95
  // 8,16,30   - ping ~95
  // 8,4,30    - ping ~110
  // 8,64,30   - ping ~95
  // 4,16,60   - ping ~90
  // 1,16,60   - ping ~85
  // 16,16,60  - ping ~95
  this->name = name;
  emitter = getEmitter("emitter");
  emitter->setChannel(channelGeneral);
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  receiver->setChannel(channelGeneral);
  distanceSensorSonarRight = getDistanceSensor("Sonar/Right");
  distanceSensorSonarLeft = getDistanceSensor("Sonar/Left");
  positionSensorHeadYawS = getPositionSensor("HeadYawS"); // is there an x and y?... !!! right now using this is as x
  positionSensorHeadPitchS = getPositionSensor("HeadPitchS"); // using this as y
  positionSensor3 = getPositionSensor("RShoulderPitchS");
  positionSensor4 = getPositionSensor("RShoulderRollS");
  distanceSensorSonarRight->enable(timeStep);
  distanceSensorSonarLeft->enable(timeStep);
  gps = new GPS("gps");
  gps->enable(100);
}

long NaoRobot::getTime()
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
  
  int counter = 0;
  size_t MessageID = 1;
  size_t channel = channelGeneral;
  long now;
  double distanceValSonarRight, distanceValSonarLeft, positionValHeadYawS, positionValHeadPitchS;
  while(step(timeStep) != -1)
  {
    counter++;
    receiver->setChannel(channel);
    channel = (channel == channelGeneral ? channelSupervisor : channelGeneral); // doesnt work...
    // not sure why changing channel isnt seeming to make a difference...
    //cout << counter << endl;
    if (counter == 20)
      counter = 0;
  
    distanceValSonarRight = distanceSensorSonarRight->getValue(); //!!! using this as "x"... !!!
    distanceValSonarLeft  = distanceSensorSonarLeft->getValue(); //!!! using this as "y"... !!!
    positionValHeadYawS = positionSensorHeadYawS->getValue();
    positionValHeadPitchS = positionSensorHeadPitchS->getValue();
    const double* loc = gps->getValues();
    
    now = getTime();
    
    Data dataSending(MessageID, name, now, loc[0], loc[1], loc[2], 3, 4, 5); //!!! just using constants for velocity right now. please set
    if (counter == 0)
      cout << sName << " sending:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName() << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.z << " " << dataSending.velocityX << " " << dataSending.velocityY << " " << dataSending.velocityZ << endl;
    emitter->send(&dataSending, sizeof(dataSending));
    MessageID++;
    if (receiver->getQueueLength()>0){
      for (int i = 0; i <receiver-> getQueueLength(); i++)
      {
        const Data* d = (const Data*) receiver->getData();
        
        now = getTime();
        long ping = (now - d->time);
          
        if (counter == 0) //print every few steps instead of at each step
          cout << sName << " received: (" << now << "): " << d->messageID << " " << d->getName()  << " " << d->time << " " << d->x << " " << d->y <<  " " << d->z << " " << d->velocityX << " " << d->velocityY << " " << d->velocityZ << " " << d->getMessage() << "; ping: " << ping << "ms" << endl;
        //if (counter % 2 == 0)
        //  delete d; 
        receiver->nextPacket();
      }
    }
    //cout << "channel (below): " << receiver->getChannel() << " " << channel << endl;
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


  delete robot;
  return 0;
}
