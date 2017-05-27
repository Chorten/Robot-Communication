// File:          NaoController.cpp
// Date:
// Description:
// Author:
// Modifications:

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
#include <webots/Gyro.hpp>
#include <webots/Compass.hpp>
#include <webots/InertialUnit.hpp>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <../Data.h>
#include <../NaoRobot2.h>
#include <../Ball2.h>
#include <cmath>
#include <vector>
#include <algorithm>

#define NE "NE"  
#define SE "SE" 
#define NW "NW" 
#define SW "SW" 
#define NORTH "NORTH" 
#define SOUTH "SOUTH" 
#define EAST "EAST" 
#define WEST "WEST" 
#define PI 3.14159265

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

string motionHandWave = "../motions/HandWave.motion";
string motionForward = "../motions/Forwards.motion";
string motionForward50 = "../motions/Forwards50.motion";
string right60 = "../motions/TurnRight60.motion";
string right40 = "../motions/TurnRight40.motion";
string left40 = "../motions/TurnLeft40.motion";
string left60 = "../motions/TurnLeft60.motion";
string left180 = "../motions/TurnLeft180.motion";
string sideStepLeft = "../motions/SideStepLeft.motion";
string sideStepRight =  "../motions/SideStepRight.motion";

// team blue are those without a jersey and numbers are less than 1000
// team red are those with a red jersey and numbers are  between 1000 to 2000
vector<string> robotNames = {"0001", "0002", "1001"}; 
vector<string> robotNamesIgnore = {"ball", "boss"};

class NaoRobot : public Robot
{
  public:
    static const size_t nameSize = 4;
    NaoRobot(char* name);
    void run();
    double getAngle(double myPosX, double myPosZ, double ball_x, double ball_y); 
    void rotate(double angle, string myDirect, string ballDirect); 
    void move(string filename, bool loop, bool sync); 
    void setMotion(double angle, string ballDirect); 
    string ballQuadrant(double x, double y, double myPosX, double myPosZ); 
    double setMyDirection(double anglex, double angley);
    bool ClosestToBall();
    double getRobotAngle(double angleX, double angleY);
    double getRobotFacingBallAngle(double robotX, double robotY, double ballX, double ballY);
    void turnRobot(double initialAngle, double finalAngle);
    
  private:
    int timeStep;
    char* name;
    Receiver* receiver;
    Emitter* emitter;
    GPS* gps;
    Gyro* gyro;
    Compass* compass;
    //InertialUnit* inertialUnit;
    Ball2 *newball;
    NaoRobot2* me;
    vector<NaoRobot2*> robots;
    long GetTime();
    string myDirection;
    string BallDirection;
    Motion* motion; 
    double myPosX; 
    double myPosZ; 
    double myAngle; 
    
    void UpdateRobot(const Data* data);
    void UpdateSelf(Data data);
    double Distance(double x1, double z1, double x2, double z2);
};

NaoRobot::NaoRobot(char* name)
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
  emitter = getEmitter("emitter");
  emitter->setChannel(channelGeneral);
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  receiver->setChannel(channelGeneral);
  gps = new GPS("gps");
  gyro = new Gyro("gyro");
  gps->enable(timeStep);
  gyro->enable(timeStep);
  compass = new Compass("compass");
  compass->enable(timeStep);
  //inertialUnit = new InertialUnit("inertial unit");
  //inertialUnit->enable(timeStep);
}

long NaoRobot::GetTime()
{
  auto nowC = high_resolution_clock::now();
  long now = duration_cast<milliseconds>(nowC.time_since_epoch()).count();
  return now;
}

void NaoRobot::move(string filename, bool loop = false, bool sync = true)
// sync is default by true! so thread will sleep and will not gather data from sensors or anything
// possible 
{
  if (loop) sync = false;
  
  //check if last motion is still playing, if it is wait
  //do {
  //  step(timeStep);
  //}
  //while (!motion->isOver());
  
  motion = new Motion(filename);
  if (! motion->isValid())
  {
    cout << "could not load file: " << filename << std::endl;
    delete motion;
  }
  motion->setLoop(loop);
  motion->play();
  cout << "MOTION " << filename << " STARTED\n";
  if (sync)
    do {
      step(timeStep);
    }
    while (!motion->isOver());
  cout << "MOTION " << filename << " STOPPED\n";

  //motion->setLoop(false);
}

double NaoRobot::getAngle(double myPosX, double myPosZ, double ball_x, double ball_y)
{
  return abs(atan((ball_x - myPosX)/(ball_y - myPosZ))*180 / PI);
}


void NaoRobot::rotate(double angle, string myDirect, string ballDirect)
{
  cout<<"Angle: "<<angle<<" My direction: "<<myDirect<<endl;
  if(myDirect == NORTH)
  {
    if(ballDirect == NE || ballDirect == NW)
    {
      setMotion(angle, ballDirect);
    }
    else if(ballDirect == SE || ballDirect == SW)
    {
      angle = 180 - angle;
      setMotion(angle, ballDirect);
    }
  }
  else if(myDirect == SOUTH)
  {
    if(ballDirect == NE || ballDirect == NW)
    {
      angle = 180 - angle;
      setMotion(angle, ballDirect);
    } 
    else if(ballDirect == SE || ballDirect == SW)
    {  
      setMotion(angle, ballDirect);
    }
    
  }
  else if(myDirect == EAST)
  {
    if(ballDirect == NE || ballDirect == NW)
    {
      angle = 90 - angle;
      setMotion(angle, ballDirect);
    } 
    else if(ballDirect == SE || ballDirect == SW)
    {  
      angle = 90 + angle;
      setMotion(angle, ballDirect);
    }
    
  }
  else if(myDirect == WEST)
  {
    if(ballDirect == NE || ballDirect == NW)
    {
      angle = 90 + angle;
      setMotion(angle, ballDirect);
    } 
    else if(ballDirect == SE || ballDirect == SW)
    {  
      angle = 90 - angle;
      setMotion(angle, ballDirect);
    }
    
  }
}

void NaoRobot::setMotion(double angle, string ballDirect)
{
  if(angle< 30)
  {
    cout << "ANGLE IS LESS THAN 30!! walking...\n";
    move(motionForward);
  }
  else if(angle>=30 && angle <50)//40
  {
    if(myDirection == NORTH){
        if(ballDirect == NE) move(right40);
        if(ballDirect == NW) move(left40);
    }
    else if(myDirection == SOUTH){
        if(ballDirect == SE) move(left40);
        if(ballDirect == SW) move(right40);
    }
    else if(myDirection == EAST){
        if(ballDirect == NE) move(left40);
        if(ballDirect == SE) move(right40);
    }
    else if(myDirection == WEST){
        if(ballDirect == NW) move(right40);
        if(ballDirect == SW) move(left40);
    }
  }
  else if(angle>=50 && angle <70)
  {
    if(myDirection == NORTH){
        if(ballDirect == NE) move(right60);
        if(ballDirect == NW) move(left60);
    }
    else if(myDirection == SOUTH){
        if(ballDirect == SE) move(left60);
        if(ballDirect == SW) move(right60);
    }
    else if(myDirection == EAST){
        if(ballDirect == NE) move(left60);
        if(ballDirect == SE) move(right60);
    }
    else if(myDirection == WEST){
        if(ballDirect == NW) move(right60);
        if(ballDirect == SW) move(left60);
    }
  }
  else if(angle >= 70)
  {
    if(myDirection == NORTH){
        if(ballDirect == NE||ballDirect == SE) move(right60);
        if(ballDirect == NW||ballDirect == SW) move(left60);
    }
    else if(myDirection == SOUTH){
        if(ballDirect == SE||ballDirect == NE) move(left60);
        if(ballDirect == SW||ballDirect == NW) move(right60);
    }
    else if(myDirection == EAST){
        if(ballDirect == NE||ballDirect == NW) move(left60);
        if(ballDirect == SE||ballDirect == SW) move(right60);
    }
    else if(myDirection == WEST){
        if(ballDirect == NW||ballDirect == NE) move(right60);
        if(ballDirect == SW||ballDirect == SE) move(left60);
    }
    angle-=60;
    setMotion(angle,ballDirect); 
  }
  
}

string NaoRobot::ballQuadrant(double x, double y, double myPosX, double myPosZ)
{
  cout<<"ballQuadrant: "<<x<<" "<<y<<" "<<myPosX<<" "<<myPosZ<<endl;
  if(x>=myPosX)
  {
    cout<<" ball here 1"<<endl;
    if(y>=myPosZ) return NE;
    else if (y<myPosZ) return SE;
  }
  else if(x<myPosZ)
  {
    cout<<" ball here 2"<<endl;
    if(y>=myPosZ) return NW;
    else if (y<myPosZ) return SW;
  }
  else return "Error";
}

double NaoRobot::setMyDirection(double anglex, double angley)
{
  double rad = atan2(angley,anglex) *180/PI; // had as x/y
  //return rad < 0 ? rad + 360 : rad;
  
  if(rad>=0.0)
  {
    if(rad<=45.0) rad = rad + 360;
    if(rad >45 && rad <= 135) myDirection = NORTH;
    if(rad >135 && rad <= 225) myDirection = WEST;
    if(rad > 225 && rad <= 315) myDirection = SOUTH;
    if(rad>315 && rad <=405) myDirection = EAST;
  }
  else
  {
    if(rad >= -45.0) rad = rad - 360;
    if(rad < -45 && rad >= -135) myDirection = SOUTH;
    if(rad < -135 && rad >= -225) myDirection = WEST;
    if(rad < -225 && rad >= -315) myDirection = NORTH;
    if(rad < -315 && rad >= -405) myDirection = EAST;
  }
  return rad;
}

double NaoRobot::getRobotAngle(double angleX, double angleY)
{
  double rad = atan2(angleY,angleX) *180/PI;
  return rad < 0 ? rad + 360 : rad;
}

double NaoRobot::getRobotFacingBallAngle(double robotX, double robotY, double ballX, double ballY)
{
  double rad = atan2(ballY - robotY, ballX - robotX) *180/PI;
  return rad < 0 ? rad + 360 : rad;
}

void NaoRobot::turnRobot(double initialAngle, double finalAngle)
{
  double turnAngle = finalAngle - initialAngle;
  if (turnAngle > 0) // if positive turn left
  {
    if(turnAngle< 30)
      cout << "ANGLE IS LESS THAN 30!!\n";
    else if(turnAngle>=30 && turnAngle<50)
      move(left40);
    else if(turnAngle>=50 && turnAngle <70)
      move(left60);
    else
    {
      move(left60);
      turnRobot(initialAngle - 60, finalAngle);
    }
  }
  else // if negative turn right
  {
    if(-turnAngle< 30)
      cout << "ANGLE IS LESS THAN 30!!\n";
    else if(-turnAngle>=30 && -turnAngle<50)
      move(right40);
    else if(-turnAngle>=50 && -turnAngle <70)
      move(right60);
    else
    {
      move(right60);
      turnRobot(initialAngle - 60, finalAngle);
    }
  }  
}

double NaoRobot::Distance(double x1, double z1, double x2, double z2)
{
    return sqrt(pow((x2 - x1), 2) + pow((z2 - z1), 2));
}


bool NaoRobot::ClosestToBall()
{
  // check distance from ball for self
  double myDistance = Distance(me->x, me->z, newball->getx(), newball->getz());
  
  //cout << "My distance: " << myDistance << "; x, z: " << me->x << me->z << " ; ball x, z" << newball->getx() << " " << newball->getz() << endl;
  if (myDistance > 9999 || myDistance < -9999 || myDistance != myDistance) // has not finished initializing
    return false;
  
  // check distance from ball for each robot
  for(int i = 0; i < robots.size(); i++)
  {
    double distanceI = Distance(robots[i]->x, robots[i]->z, newball->getx(), newball->getz());
    //cout << robots[i]->name + "'s distance to ball: " << distanceI << endl;
    if (distanceI > 9999 || distanceI < -9999 || distanceI != distanceI) // has not finished initializing // !!! not working properly...
      return false;
    if(distanceI < myDistance)
      return false;
    else if(distanceI == myDistance)
      try
      {
        cout << "SAME DISTANCE!!!" << endl;
        if (stoi(me->name) < stoi(robots[i]->name))
        {
          cout << "FALSE!" << endl;
          return false;
        }
      }
      catch (...)
      {
        cout << "stoi failed" << endl;
      }
  }
  return true;
}

void NaoRobot::UpdateRobot(const Data* data)
{
  // find robot in list by name
  if (find(robotNamesIgnore.begin(), robotNamesIgnore.end(),data->getName())!=robotNamesIgnore.end())
    {
      return;
    }
  for(int i = 0; i < robots.size(); i++)
    if (robots[i]->name == data->getName())
    {
      //update fields
      robots[i]->role = data->role;
      robots[i]->x = data->x;
      robots[i]->y = data->y;
      robots[i]->z = data->z;
      robots[i]->velocityX = data->velocityX;
      robots[i]->velocityY = data->velocityY;
      robots[i]->velocityZ = data->velocityZ;
      return;
    }
  cout << "Ignoring update to robot " + data->getName() + ". Robot does not exist in list. Make sure to add robot name to robotNames global variable\n";
}

void NaoRobot::UpdateSelf(Data data)
{
  // find robot in list by name
    if (me->name == data.getName())
    {
      //update fields
      me->role = data.role;
      me->x = data.x;
      me->y = data.y;
      me->z = data.z;
      me->velocityX = data.velocityX;
      me->velocityY = data.velocityY;
      me->velocityZ = data.velocityZ;
    }
    else
      cout << "Ignoring update to self: Data for self was not passed...\n";
}

void NaoRobot::run()
{
  string sName(name);
  me = new NaoRobot2(sName);
  
  int counter = 0;
  //bool init = false;
  size_t MessageID = 1;
  size_t channel = channelGeneral;
  newball = new Ball2();
  long now, ping;
  const Data* d;
  const double* loc;
  const double* speed;
  const double* angles;
  const double* inertia;
  
  for(int i = 0; i < robotNames.size(); i++)
    if (robotNames[i] != string(name))
    {
      robots.push_back(new NaoRobot2(robotNames[i]));
    }
    
  //move(motionForward, false);
  
    
  while(step(timeStep) != -1)
  {
    counter++;
    receiver->setChannel(channel);
    channel = (channel == channelGeneral ? channelSupervisor : channelGeneral); // doesnt work...
    if (counter == 20)
    {
      counter = 0;
      //init = true;
    }
  
    loc = gps->getValues();
    speed = gyro->getValues();
    angles = compass->getValues();
    //inertia = inertialUnit->getRollPitchYaw();
    
    myPosX = loc[0]; // !!!c - was loc[2]
    myPosZ = loc[2]; // !!!c - was loc[0]
    
    now = GetTime();
    
    Data dataSending(MessageID, name, now, roleUndefined, loc[0], loc[1], loc[2], speed[0], speed[1], speed[2]);
    if (counter == 0)
      cout << sName << " sending:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName()
        << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.z << " "
        << dataSending.velocityX << " " << dataSending.velocityY << " " << dataSending.velocityZ << " angles: " << angles[0]
        << " " << angles[1] << " " << angles[2] << endl;
       //<< " inertia: " << inertia[0] << " " << inertia[1] << " " << inertia[2] << endl;
    emitter->send(&dataSending, sizeof(dataSending));
    
    UpdateSelf(dataSending);
    
    MessageID++;
    if (receiver->getQueueLength()>0)
    {
      if (counter == 0)
        cout << "reciever  queue length: " << receiver->getQueueLength() << endl;
      while(receiver-> getQueueLength())
      {
        d = (const Data*) receiver->getData();
        UpdateRobot(d);
        
        now = GetTime();
        ping = (now - d->time);
          
        if (counter == 0) //print every few steps instead of at each step
          cout << sName << " received: (" << now << "): " << d->messageID << " " << d->getName()  << " " << d->time << " " << d->x << " " << d->y <<  " " << d->z << " " << d->velocityX << " " << d->velocityY << " " << d->velocityZ << " " << d->getMessage() << "; ping: " << ping << "ms" << endl;

        if(d->getName() == "ball")
        {
            newball->setPos(d->x, d->z); // !!! isnt it d->z, d->x
            
            //BallDirection = ballQuadrant(newball->getx(), newball->getz(), myPosX, myPosZ);

            //myAngle = getAngle(myPosX, myPosZ, newball->getx(), newball->getz());
            //rotate(myAngle,myDirection, BallDirection);
            //move(motionForward); //C!!!
            if(counter == 0)
            {
              //cout<<"My position: "<<myPosX <<" "<<myPosZ<<endl;
              //cout<< "Ball's position: "<<newball->getx()<<" "<<newball->getz()<<endl;
              //cout<<"Ball's direction: "<<BallDirection<<endl;
            }
        }
        receiver->nextPacket();
      }
    }
    
    // process data
    bool close = ClosestToBall();
    //cout << me->name << " close to ball: " << close << endl;
    if (close) // if you are closest to ball
    {
      // calculate 
      double myAngle = getRobotAngle(angles[1], angles[0]);
      double myFinalAngle = getRobotFacingBallAngle(me->z, me->x, newball->getz(), newball->getx());
      if (counter == 0)
      {
        cout << "robot angle (" << sName << "): " << myAngle << "   " << angles[1] << " " << angles[0] << endl;
        cout << "robot should face " << myFinalAngle << "   " << me->z << " " << me->x << " " << newball->getz() << " " << newball->getx() << endl;
      }
      // now actually turn and then walk
      turnRobot(myAngle, myFinalAngle);
      move(motionForward);
      counter = -1;
    }
      
  }   
}

int main(int argc, char **argv)
{
  char* name = argv[1];
  // create the Robot instance.
  NaoRobot *robot = new NaoRobot(name);
  
  robot->run();

  delete robot;
  return 0;
}
