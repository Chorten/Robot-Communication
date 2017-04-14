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
  channelGeneral = 2
};

class SupervisorServer : public Supervisor
{
  public:
    SupervisorServer();
    void run();
  private:
    int timeStep;
    Receiver* receiver;
    Emitter* emitter;
    long getTime();
};

SupervisorServer::SupervisorServer()
{
  timeStep = 16;
  emitter = getEmitter("emitter");
  emitter->setChannel(channelGeneral);
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  receiver->setChannel(channelGeneral);
}

long SupervisorServer::getTime()
{
  //std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  auto nowC = high_resolution_clock::now();
  long now = duration_cast<milliseconds>(nowC.time_since_epoch()).count();
  return now;
}

void SupervisorServer::run()
{
  int counter = 0;
  size_t messageID = 1;
  long now;
  // SUPERVISOR DOES NOT NEED TO RELY ON DATA SENT FROM ROBOTS TO CARRY OUT ACTIONS
  // USE THE BELOW TO DIRECTLY GET LOCATIONS
  Node* ball_node = getFromDef("ball");
  Field* ball_position = ball_node->getField("translation");

  while(step(timeStep) != -1)
  {
    counter++;
    //cout << counter << endl;
    if (counter == 20)
      counter = 0;
      
    now = getTime();
    
    // THIS IS SUPERVISOR GETTING BALL LOCATION
    
    const double* pos = ball_position->getSFVec3f();
    //cout << "position of BALL: " << endl;
    // SUPERVISOR DISPLAYING BALL POSITION
    //cout << pos[0] << pos[1] << pos[2] << endl;
    Data dataSending(messageID, "boss", now, 0, 0, 0, 0, 0, 0, "hello there! your supervisor here!", 34);
    if (counter == 0)
      cout << "server" << " sending:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName() << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.z << " " << dataSending.velocityX << " " << dataSending.velocityY << " " << dataSending.velocityZ << " " << dataSending.getMessage() << endl;
    
    emitter->send(&dataSending, sizeof(dataSending));
    messageID++;
    
    if (receiver->getQueueLength()>0){ // will actually have to loop through the queue
      for (int i = 0; i <receiver-> getQueueLength(); i++)
      {
        const Data* d = (const Data*) receiver->getData();
        
        now = getTime();
        long ping = (now - d->time);
          
        if (counter == 0) //print every few steps instead of at each step
          cout << "server" << " received: (" << now << "): " << d->messageID  << " " << d->getName()  << " " << d->time << " " << d->x << " " << d->y << " " << d->z << " " << d->velocityX << " " << d->velocityY << " " << d->velocityZ << " " << d->getMessage() << "; ping: " << ping << "ms" << endl;
        //if (counter % 2 == 0)
        //  delete d; 
        receiver->nextPacket();
      }
    }
  }
}

int main(int argc, char **argv)
{
  SupervisorServer *server= new SupervisorServer();

  server->run();

  // Enter here exit cleanup code.

  delete server;
  return 0;
}
