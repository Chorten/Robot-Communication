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
#include <cmath>

//#else
//#include </home/viruszer0/Desktop/Webots/include/controller/c/webots/differential_wheels.h>
//#endif

using namespace webots;
using namespace std;
using namespace chrono;

string filename1 = "/home/chorten/Documents/Senior-Design-Project/controllers/motions/HandWave.motion";
string filename2 = "/home/chorten/Documents/Senior-Design-Project/controllers/motions/Forwards.motion";
string right60 = "/home/chorten/Documents/Senior-Design-Project/controllers/motions/TurnRight60.motion";
string right40 = "/home/chorten/Documents/Senior-Design-Project/controllers/motions/TurnRight40.motion";
string left40 = "/home/chorten/Documents/Senior-Design-Project/controllers/motions/TurnLeft40.motion";
string left60 = "/home/chorten/Documents/Senior-Design-Project/controllers/motions/TurnLeft60.motion";
string left180 = "/home/chorten/Documents/Senior-Design-Project/controllers/motions/TurnLeft180.motion";

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
    bool Turn(double* loc,Ball *obj); //return true if angle is set
    void Move();
    void run();
    class Ball
      {
        private:
          double ball_x;
          double ball_y;
          string id;
        public:
          double getx(){return ball_x;}
          double gety(){return ball_y;}
          void setPos(double x, double y){
            ball_x = x;
            ball_y = y;
          }
          string getid(){
            id = "ball";
            return id;
          }
       };

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

bool NaoRobot::Turn(double* loc, Ball *obj)
{
  double ballPosX = obj->getx();
  double ballPosY = obj->gety();
  double myPosX = loc[0];
  double myPosY = loc[1];
  double result; //acos
  string rotate;
  result = atan((ballPosX - myPosX)/(ballPosY - myPosY));
  //Assuming a 20 degree freedom
  //if(result >0.0 && result <20.0) rotate = ;
  Motion *turn = new Motion(rotate);
  if (! turn->isValid())
  {
    cout << "could not load file: " << filename2 << std::endl;
    delete walk;
  }
  turn->setLoop(true);
  turn->play();
  turn->setLoop(false);
  return true;
}


void NaoRobot::Move()
{
  Motion *walk = new Motion(filename2);
  if (! walk->isValid())
  {
    cout << "could not load file: " << filename2 << std::endl;
    delete walk;
  }
  walk->setLoop(true);
  walk->play();
  
}

void NaoRobot::run()
{
  Ball *newball = new Ball();
  string sName(name);
  
  int counter = 0;
  size_t MessageID = 1;
  size_t channel = channelGeneral;
  long now;
  double distanceValSonarRight, distanceValSonarLeft, positionValHeadYawS, positionValHeadPitchS;
  while(step(timeStep) != -1)
  {
    Move();
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
        cout<<"Ball : "<< d->getName()<<endl;
        if(d->getName() == "ball")
        {
          //newball->setPos(d->x, d->y);
          cout << "\nBall's new position "<< newball->getx() <<" " <<newball->gety()<<endl;
          //while(!Turn(loc,newball)) continue; //return true if angle is set
        }
        
        receiver->nextPacket();
      }
      //check for ball location
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
