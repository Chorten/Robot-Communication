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

string filename1 = "../motions/HandWave.motion"; 
string filename2 = "../motions/Forwards.motion"; 
string right60 = "../motions/TurnRight60.motion"; 
string right40 = "../motions/TurnRight40.motion"; 
string left40 = "../motions/TurnLeft40.motion"; 
string left60 = "../motions/TurnLeft60.motion"; 
string left180 = "../motions/TurnLeft180.motion";

vector<string> robotNames = {"0001", "0002", "0003"};
vector<string> robotNamesIgnore = {"ball", "boss"};

class NaoRobot : public Robot
{
  public:
    class Ball2
    {
        private:
          double ball_x;
          double ball_z;
          string id;
        public:
          double getx(){return ball_x;}
          double getz(){return ball_z;}
          void setPos(double x, double z){
            ball_x = x;
            ball_z = z;
          }
          string getid(){
            id = "ball";
            return id;
          }
    };
    static const size_t nameSize = 4;
    NaoRobot(char* name);
    void run();
    double getAngle(double myPosX, double myPosY, double ball_x, double ball_y); 
    void rotate(double angle, string myDirect, string ballDirect); 
    void move(string filename, bool loop); 
    void setMotion(double angle, string ballDirect); 
    string ballQuadrant(double x, double y, double myPosX, double myPosY); 
    void setMyDirection(double anglex, double angley);
    bool ClosestToBall();
    
  private:
    int timeStep;
    char* name;
    Receiver* receiver;
    Emitter* emitter;
    GPS* gps;
    Gyro* gyro;
    Compass* compass;
    InertialUnit* inertialUnit;
    Ball2 *newball;
    NaoRobot2* me;
    vector<NaoRobot2*> robots;
    long GetTime();
    string myDirection;
    string BallDirection;
    Motion* walk; 
    double myPosX; 
    double myPosY; 
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
  gps = new GPS("gps");
  gyro = new Gyro("gyro");
  gps->enable(100);
  gyro->enable(100);
  compass = new Compass("compass");
  compass->enable(100);
  inertialUnit = new InertialUnit("inertial unit");
  inertialUnit->enable(100);
}

long NaoRobot::GetTime()
{
  auto nowC = high_resolution_clock::now();
  long now = duration_cast<milliseconds>(nowC.time_since_epoch()).count();
  return now;
}

void NaoRobot::move(string filename, bool loop = false)
{
  walk = new Motion(filename);
  if (! walk->isValid())
  {
    cout << "could not load file: " << filename << std::endl;
    delete walk;
  }
  walk->setLoop(loop);
  walk->play();
  //walk->setLoop(false);
}

double NaoRobot::getAngle(double myPosX, double myPosY, double ball_x, double ball_y)
{
  return abs(atan((ball_x - myPosX)/(ball_y - myPosY))*180 / PI);
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
    cout << "ANGLE IS LESS THAN 30!!\n";
    move(filename2);
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

string NaoRobot::ballQuadrant(double x, double y, double myPosX, double myPosY)
{
  cout<<"ballQuadrant: "<<x<<" "<<y<<" "<<myPosX<<" "<<myPosY<<endl;
  if(x>=myPosX)
  {
    cout<<" ball here 1"<<endl;
    if(y>=myPosY) return NE;
    else if (y<myPosY) return SE;
  }
  else if(x<myPosY)
  {
    cout<<" ball here 2"<<endl;
    if(y>=myPosY) return NW;
    else if (y<myPosY) return SW;
  }
  else return "Error";
}

void NaoRobot::setMyDirection(double anglex, double angley)
{
  double rad = atan(anglex/angley) *180/PI;
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
}

double NaoRobot::Distance(double x1, double z1, double x2, double z2)
{
    return sqrt(pow((x2 - x1), 2) + pow((z2 - z1), 2));
}


bool NaoRobot::ClosestToBall()
{
  // check distance from ball for self
  double myDistance = Distance(me->x, me->z, newball->getx(), newball->getz());
  
  cout << "My distance: " << myDistance << "; x, z: " << me->x << me->z << " ; ball x, z" << newball->getx() << " " << newball->getz() << endl;
  if (myDistance > 9999 || myDistance < -9999 || myDistance != myDistance) // has not finished initializing
    return false;
  
  // check distance from ball for each robot
  for(int i = 0; i < robots.size(); i++)
  {
    double distanceI = Distance(robots[i]->x, robots[i]->z, newball->getx(), newball->getz());
    cout << robots[i]->name + "'s distance to ball: " << distanceI << endl;
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
  bool init = false;
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
    
  //move(filename2, true);
    
  while(step(timeStep) != -1)
  {
    counter++;
    receiver->setChannel(channel);
    channel = (channel == channelGeneral ? channelSupervisor : channelGeneral); // doesnt work...
    if (counter == 20)
    {
      counter = 0;
      init = true;
    }
  
    loc = gps->getValues();
    speed = gyro->getValues();
    angles = compass->getValues();
    inertia = inertialUnit->getRollPitchYaw();
    
    myPosX = loc[2]; 
    myPosY = loc[0]; 
    
    now = GetTime();
    
    Data dataSending(MessageID, name, now, roleUndefined, loc[0], loc[1], loc[2], speed[0], speed[1], speed[2]);
    if (counter == 0)
      cout << sName << " sending:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName() << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.z << " " << dataSending.velocityX << " " << dataSending.velocityY << " " << dataSending.velocityZ << " angles: " << angles[0] << " " << angles[1] << " " << angles[2] << " inertia: " << inertia[0] << " " << inertia[1] << " " << inertia[2] << endl;
    emitter->send(&dataSending, sizeof(dataSending));
    
    UpdateSelf(dataSending);
    
    MessageID++;
    if (receiver->getQueueLength()>0){
      for (int i = 0; i <receiver-> getQueueLength(); i++)
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
            
            BallDirection = ballQuadrant(newball->getx(), newball->getz(), myPosX, myPosY);
            
            setMyDirection(angles[0], angles[1]);
            myAngle = getAngle(myPosX, myPosY, newball->getx(), newball->getz());
            rotate(myAngle,myDirection, BallDirection);
            //move(filename2); //C!!!
            if(counter == 0)
            {
              cout<<"My position: "<<myPosX <<" "<<myPosY<<endl;
              cout<< "Ball's position: "<<newball->getx()<<" "<<newball->getz()<<endl;
              cout<<"Ball's direction: "<<BallDirection<<endl;
            }
          
        }
        receiver->nextPacket();
      }
    }
    
    // process data
    bool close = ClosestToBall();
    cout << me->name << " close to ball: " << close << endl;
    if (close && init)
      move(filename2, true);
      
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
