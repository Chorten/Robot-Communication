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
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <../Data.h>
#include <../NaoRobot2.h>
#include <../Ball2.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <exception>

#define PI 3.14159265
#define TEAM_RED_FORWARD_ANGLE 90
#define TEAM_BLUE_FORWARD_ANGLE 270

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

enum Status
{
  statusStandBy = 0,
  statusGoingForBallToPass = 1,
  statusGettingReadyToReceivePass = 2
  
};

enum Command
{
  commandStandBy = 0,
  commandDrillGreedy = 1,
  commandDrillCooperative = 2,
  commandDrillDeadlock = 3
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
    void move(string filename, bool loop, bool sync);
    bool ClosestToBall();
    double getRobotAngle(double angleX, double angleY);
    double getRobotFacingBallAngle(double robotX, double robotY, double ballX, double ballY);
    double getForwardDirection();
    void turnRobot(double initialAngle, double finalAngle);
    
  private:
    int timeStep;
    char* name;
    Receiver* receiver;
    Emitter* emitter;
    GPS* gps;
    Gyro* gyro;
    Compass* compass;
    Ball2 *newball;
    NaoRobot2* me;
    vector<NaoRobot2*> robots;
    Command supervisorCommand = commandStandBy;
    long GetTime();
    Motion* motion;
    double myPosX; 
    double myPosZ; 
    double myAngle; 
    
    void UpdateRobot(const Data* data);
    void UpdateSelf(Data data);
    double Distance(double x1, double z1, double x2, double z2);
    bool sameTeam(NaoRobot2* n);
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
  //cout << "MOTION " << filename << " STARTED\n";
  if (sync)
    do {
      step(timeStep);
    }
    while (!motion->isOver());
  //cout << "MOTION " << filename << " STOPPED\n";

  //motion->setLoop(false);
}

double NaoRobot::getAngle(double myPosX, double myPosZ, double ball_x, double ball_y)
{
  return abs(atan((ball_x - myPosX)/(ball_y - myPosZ))*180 / PI);
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

double NaoRobot::getForwardDirection()
{
  if (name[0] == '0')
    return TEAM_RED_FORWARD_ANGLE;
  else
    return TEAM_BLUE_FORWARD_ANGLE;
}

void NaoRobot::turnRobot(double initialAngle, double finalAngle)
{
  double turnAngle = finalAngle - initialAngle;
  if (initialAngle < 0)
    initialAngle = initialAngle + 360;
  if (turnAngle < 0)
    turnAngle = turnAngle + 360;
  if (turnAngle < 180) // turn left
  {
    if(turnAngle< 30)
      {}
    else if(turnAngle>=30 && turnAngle<50)
      move(left40);
    else if(turnAngle>=50 && turnAngle <70)
      move(left60);
    else
    {
      move(left60);
      //turnRobot(initialAngle + 60, finalAngle);
      // don't actually do recursive call, better to return control and check if the robot should still turn
    }
  }
  else // turn right
  {
    turnAngle = 360 - turnAngle;
    if(turnAngle< 30)
      {}
    else if(turnAngle>=30 && turnAngle<50)
      move(right40);
    else if(turnAngle>=50 && turnAngle <70)
      move(right60);
    else
    {
      move(right60);
      //turnRobot(initialAngle - 60, finalAngle);
      // don't actually do recursive call, better to return control and check if the robot should still turn
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
  
  // check distance from ball for each robot on the same team
  for(int i = 0; i < robots.size(); i++)
  {
    if (sameTeam(robots[i]))
    {
      double distanceI = Distance(robots[i]->x, robots[i]->z, newball->getx(), newball->getz());
      //cout << robots[i]->name + "'s distance to ball: " << distanceI << endl;
      if (distanceI > 9999 || distanceI < -9999 || distanceI != distanceI) // has not finished initializing
        return false;
      if(distanceI < myDistance)
        return false;
      else if(distanceI == myDistance)
      {
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
    }
  }
  return true;
}

bool NaoRobot::sameTeam(NaoRobot2* n)
{
  try {
    return name[0] == n->name[0];
  }
  catch (exception& e) {
    cout << e.what() << endl;
    return false;
  }
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
  bool robotsInit = false;
  bool ballInit = false;
  size_t MessageID = 1;
  size_t channel = channelGeneral;
  newball = new Ball2();
  long now, ping;
  const Data* d;
  const double* loc;
  const double* speed;
  const double* angles;
  
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
    
    if (counter == timeStep)
      counter = 0;

    now = GetTime();
    
    //recieve data
    MessageID++;
    if (receiver->getQueueLength()>0)
    {
      if (counter == -1)
        cout << "reciever  queue length: " << receiver->getQueueLength() << endl;
      while(receiver-> getQueueLength())
      {
        d = (const Data*) receiver->getData();
        UpdateRobot(d);
        if (d->getName() == "boss")
        {
          supervisorCommand = (Command)d->command;
          cout << "recieved command from supervisor\n";
        }
        
        now = GetTime();
        ping = (now - d->time);
          
        if (counter == -1) //print every few steps instead of at each step
          cout << sName << " received: (" << now << "): " << d->messageID << " " << d->getName()  << " " << d->time << " " << d->x << " " << d->y <<  " " << d->z << " " << d->velocityX << " " << d->velocityY << " " << d->velocityZ << " " << d->getMessage() << " " << d->command << " ; ping: " << ping << "ms" << endl;

        if(d->getName() == "ball")
        {
            newball->setPos(d->x, d->z); // !!! isnt it d->z, d->x
            ballInit = true;
        }
        receiver->nextPacket();
      }
    }
    
    // if supervisor commands to standby
    if (supervisorCommand == commandStandBy)
      continue;
    
    // send data    
    loc = gps->getValues();
    speed = gyro->getValues();
    angles = compass->getValues();
    myPosX = loc[0];
    myPosZ = loc[2]; 
    
    Data dataSending(MessageID, name, now, roleUndefined, loc[0], loc[1], loc[2], speed[0], speed[1], speed[2]);
    if (counter == -1)
      cout << sName << " sending:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName()
        << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.z << " "
        << dataSending.velocityX << " " << dataSending.velocityY << " " << dataSending.velocityZ << " angles: " << angles[0]
        << " " << angles[1] << " " << angles[2] << endl;
    emitter->send(&dataSending, sizeof(dataSending));
    
    UpdateSelf(dataSending);
    
    // wait for everything to be initialzied before making a decision
    if (!ballInit)
      continue;
    if (!robotsInit)
    {
      for(int i = 0; i < robots.size(); i++)
        if (robots[i]->role == -1)
          continue;
      robotsInit = true;
    }
    
    // process data
    bool close = ClosestToBall();
    //cout << me->name << " close to ball: " << close << endl;
    if (close) // if you are closest to ball
    {
      // calculate 
      double myAngle = getRobotAngle(angles[1], angles[0]);
      //cout << me->z << " " << me->x << " " << newball->getz() << " " << newball->getx() << endl;
      double myFinalAngle = getRobotFacingBallAngle(me->z, me->x, newball->getz(), newball->getx());
      if (true)
      {
        cout << "robot angle (" << sName << "): " << myAngle << "   " << angles[1] << " " << angles[0] << endl;
        cout << "robot should face " << myFinalAngle << "   " << me->z << " " << me->x << " " << newball->getz() << " " << newball->getx() << endl;
      }
      // now actually turn and then walk
      turnRobot(myAngle, myFinalAngle);
      move(motionForward);
      counter = -1;
    }
    else // get reaqdy for a pass
    {
      double myAngle = getRobotAngle(angles[1], angles[0]);
      turnRobot(myAngle, getForwardDirection());
      move(motionForward);
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
