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
//#include <webots/HandWave.motion>


/* TODO:  - Wrap emitter & receiver into functions
          - map sensors to real values
          - put receiver in for loop
          - use chrono for time so it is OS independent
          - ping appears to get progressively large?...
          - name does not apear when in recieved message because should use copy instead of memcpy because copy constructor is not called
          - should probably recieve and then send (right now other way around)
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
#include <sys/time.h>
#include <unistd.h>
#include <cstring>
//#include <ctime>
//#include <chrono>

//#else
//#include </home/viruszer0/Desktop/Webots/include/controller/c/webots/differential_wheels.h>
//#endif

using namespace webots;
using namespace std;

typedef unsigned char byte;

class NaoRobot : public Robot
{
  public:
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
    long getTime();
};

class Data
{
  public:
    char* name; //pointer is fine because we are never changing the name.
    long time; //time in microseconds
    double x;
    double y;
    double velocityX;
    double velocityY;
    
    Data(char* name = NULL, long time = 0, double x = 0, double y = 0, double velocityX = 0, double velocityY = 0);
};

Data::Data(char* name, long time, double x, double y, double velocityX, double velocityY)
{
  this->name = name;
  this->time = time; //time in microseconds
  this->x = x;
  this->y = y;
  this->velocityX = velocityX;
  this->velocityY = velocityY;
}

NaoRobot::NaoRobot(char* name)
{
  timeStep = 32;
  this->name = name;
  emitter = getEmitter("emitter");
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  distanceSensorSonarRight = getDistanceSensor("Sonar/Right");
  distanceSensorSonarLeft = getDistanceSensor("Sonar/Left");
  positionSensorHeadYawS = getPositionSensor("HeadYawS"); // is there an x and y?... !!! right now using this is as x
  positionSensorHeadPitchS = getPositionSensor("HeadPitchS"); // using this as y
  positionSensor3 = getPositionSensor("RShoulderPitchS");
  positionSensor4 = getPositionSensor("RShoulderRollS");
  distanceSensorSonarRight->enable(timeStep);
  distanceSensorSonarLeft->enable(timeStep);
}

long NaoRobot::getTime()
{
  timeval tv1;
  gettimeofday(&tv1, 0);
  time_t now_sec = tv1.tv_sec;
  suseconds_t now_usec = tv1.tv_usec;
  return now_sec * 1000000 + now_usec;
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

  //cout << sName << "; time: " << now_sec << " " << now_usec << endl;
  //sleep(2);
  
  //long now = getTime();
  //Data d1(name, now, 1, 2, 3, 4);
  //cout << sName << " sending:  (" << d1.time << "): " << d1.x << " " << d1.y << " " << d1.velocityX << " " << d1.velocityY << endl;
  
  //byte* byteArray = new byte[sizeof(d1)];
  //memcpy(byteArray, &d1, sizeof(d1)); //dest, source
  
  //milliseconds now = duration_cast< milliseconds >(
  //system_clock::now().time_since_epoch());
  //cout << now << endl;
  //emitter->send(messageS, 22);
  //emitter->send(byteArray, sizeof(d1));
  
  int counter = 0;
  long now;
  double distanceValSonarRight, distanceValSonarLeft, positionValHeadYawS, positionValHeadPitchS;
  while(step(timeStep) != -1)
  {
    counter++;
    if (counter == 20)
      counter = 0;
  
    distanceValSonarRight = distanceSensorSonarRight->getValue(); //!!! using this as "x"... !!!
    distanceValSonarLeft  = distanceSensorSonarLeft->getValue(); //!!! using this as "y"... !!!
    positionValHeadYawS = positionSensorHeadYawS->getValue();
    positionValHeadPitchS = positionSensorHeadPitchS->getValue();
    
    now = getTime();
    
    Data dataSending(name, now, distanceValSonarRight, distanceValSonarLeft, 3, 4); //!!! just using constatns for velocity right now. please set
    if (counter == 0)
      cout << sName << " sending:  (" << dataSending.time << "): " << dataSending.name << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.velocityX << " " << dataSending.velocityY << endl;
    byte* byteArray = new byte[sizeof(dataSending)];
    memcpy(byteArray, &dataSending, sizeof(dataSending)); //dest, source
    
    emitter->send(byteArray, sizeof(dataSending));
    
    if (receiver->getQueueLength()>0){ // will actually have to loop through the queue
      int messageSize = receiver->getDataSize();
      //string messageR((const char*)receiver->getData());
      const byte* byteArray;
      byteArray = (const byte*) receiver->getData();
      //cout << "byteArray: " << byteArray;
      
      Data dataReceived;
      memcpy(&dataReceived, byteArray, messageSize);
      
      now = getTime();
      long ping = (now - dataReceived.time) / 1000;    
        
      if (counter == 0) //print every few seconds instead of at each step
        cout << sName << " received: (" << now << "): " << " " << dataReceived.time << " " << dataReceived.x << " " << dataReceived.y << " " << dataReceived.velocityX << " " << dataReceived.velocityY << "; ping: " << ping << "ms" << endl;

      receiver->nextPacket();
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


  delete robot;
  return 0;
}
