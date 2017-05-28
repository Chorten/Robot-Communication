// File:          SupervisorController.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes

#include <webots/Supervisor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <../Data.h>

using namespace std;
using namespace webots;
using namespace chrono;

enum Channel
{
  channelSupervisor = 1,
  channelGeneral = 2,
};

enum Role
{
  roleUndefined= 0,
  roleAttacker = 1,
  roleDefender = 2,
  roleGoale = 3,
  roleNone = 4
};

enum Command
{
  commandStandBy = 0,
  commandDrillGreedy = 1,
  commandDrillCooperative = 2,
  commandDrillDeadlock = 3
};

const int RUNTIME_DrillGreedy = 20000; // 20 seconds
const int RUNTIME_DrillCooperative = 45000; // 35 seconds
const int RUNTIME_DrillDeadlock = 30000; // 30 seconds

class SupervisorServer : public Supervisor
{
  public:
    SupervisorServer();
    void run();
  private:
    int timeStep;
    Receiver* receiver;
    Emitter* emitter;
    Keyboard* keyboard;
    long getTime();
    void printMenu();
    void sendData(size_t &messageID, long now, Command command);
};

SupervisorServer::SupervisorServer()
{
  timeStep = 64;
  emitter = getEmitter("emitter");
  emitter->setChannel(channelGeneral);
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  receiver->setChannel(channelGeneral);
  keyboard = Robot::getKeyboard();
  keyboard->enable(timeStep);
}

long SupervisorServer::getTime()
{
  //std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  auto nowC = high_resolution_clock::now();
  long now = duration_cast<milliseconds>(nowC.time_since_epoch()).count();
  return now;
}

void SupervisorServer::printMenu()
{
  cout << "Welcome!\n\nMenu (will auto each in 10 seconds (simulation-time) if none is selected):\n" 
    << "1) Greedy Method Demo\n"
    << "2) Cooperative Method Demo\n"
    << "3) Deadlock Prevention Demo\n\n"
    << "Enter choice: \n";
}

void SupervisorServer::sendData(size_t &messageID, long now, Command command)
{
  Data dataSending(messageID, "boss", now, roleUndefined, 0, 0, 0, 0, 0, 0, "hello there! your supervisor here!", 34, command);
  cout << "server" << " sending:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName() << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.z << " " << dataSending.velocityX << " " << dataSending.velocityY << " " << dataSending.velocityZ << " " << dataSending.getMessage() << " " << dataSending.command << endl;
  
  emitter->send(&dataSending, sizeof(dataSending));
  messageID++;
}

void SupervisorServer::run()
{
  int counter = 0;
  int t = 0;
  size_t messageID = 1;
  Command command;
  int key = -1;
  long now;
  bool playingDemo = false;
  bool autoPlay = false;
  // SUPERVISOR DOES NOT NEED TO RELY ON DATA SENT FROM ROBOTS TO CARRY OUT ACTIONS
  // USE THE BELOW TO DIRECTLY GET LOCATIONS
  Node* node_ball = getFromDef("ball");
  Node* node_r0001 = getFromDef("r0001");
  Node* node_r0002 = getFromDef("r0002");
  Node* node_r1001 = getFromDef("r1001");
  Field* ball_position = node_ball->getField("translation");
  Field* r0001_position = node_r0001->getField("translation");
  Field* r0002_position = node_r0002->getField("translation");
  Field* r1001_position = node_r1001->getField("translation");
  Field* r0001_rotation = node_r0001->getField("rotation");
  Field* r0002_rotation = node_r0002->getField("rotation");
  Field* r1001_rotation = node_r1001->getField("rotation");
  
  printMenu();

  while(step(timeStep) != -1)
  {
    t++;
    counter++;
    //cout << counter << endl;
    if (counter == timeStep)
      counter = 0;
    if (counter == 0)
      cout << "time: " << t*64 << " " << t*64/1000 << endl;
      
    if (!playingDemo)
    {      
      key = keyboard->getKey();
      if (key != -1)
        cout << char(key) << endl;
      
      if (t * timeStep > 10000) // 10 simulation seconds have passed
      {
        autoPlay = true;
        key = (int)'2';
      }
      
      // THIS IS SUPERVISOR GETTING BALL LOCATION
      //const double* pos = ball_position->getSFVec3f();
      //cout << "position of BALL: " << endl;
      // SUPERVISOR DISPLAYING BALL POSITION
      //cout << pos[0] << pos[1] << pos[2] << endl;
      
      if (key == (int)('1'))
      {
        //command = commandDrillGreedy;
      }
      if (key == (int)('2'))
      {
        command = commandDrillCooperative;
        sendData(messageID, getTime(), command);
        cout << "Demoing the cooperative method\n\n";
        
        const double ball_pos[3]  = {2.40076, 0.096713, 1.13012};
        const double r0001_pos[3] = {1.87582, 0.334, 0.978764};
        const double r0002_pos[3] = {2.58291, 0.334, 1.9689};
        const double r1001_pos[3] = {3.03169, 0.334, 0.524922};
        const double r0001_rot[4] = {1, 0, 0, -1.5708};
        const double r0002_rot[4] = {0.983106, -0.129428, -0.129428, 4.69535};
        const double r1001_rot[4] = {0.0926918, 0.704064, 0.704061, 3.32645};
        
        ball_position->setSFVec3f(ball_pos);
        r0001_position->setSFVec3f(r0001_pos);
        r0002_position->setSFVec3f(r0002_pos);
        r1001_position->setSFVec3f(r1001_pos);
        r0001_rotation->setSFRotation(r0001_rot);
        r0002_rotation->setSFRotation(r0002_rot);
        r1001_rotation->setSFRotation(r1001_rot);
        
        // wait for demo to finish
        int demoStart = t;
        while (t * 64 < demoStart + RUNTIME_DrillCooperative)
        {
          if (t % 128 == 0)
            cout << "t : " << t << endl;
          step(timeStep);
          t++;
        }
        
        cout << "Cooperative Method Demo has finished playing!\n\n";
        
        if (autoPlay == false)
        {
          playingDemo = false;
          printMenu();
        }
        else
          key = (int)'3'; // play the next demo
      }
      if (key == (int)('3'))
      {
        command = commandDrillDeadlock;
      }
    }
    
    if (receiver->getQueueLength()>0){ // will actually have to loop through the queue
      while(receiver-> getQueueLength())
      {
        const Data* d = (const Data*) receiver->getData();
        
        now = getTime();
        long ping = (now - d->time);
          
        if (counter == -1) //print every few steps instead of at each step
          cout << "server" << " received: (" << now << "): " << d->messageID  << " " << d->getName()  << " " << d->time << " " << d->x << " " << d->y << " " << d->z << " " << d->velocityX << " " << d->velocityY << " " << d->velocityZ << " " << d->getMessage() << "; ping: " << ping << "ms" << endl;
        receiver->nextPacket();
      }
    }
  }
}

int main(int argc, char **argv)
{
  SupervisorServer *server= new SupervisorServer();

  server->run();

  delete server;
  return 0;
}
