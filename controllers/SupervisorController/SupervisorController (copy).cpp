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
#include <webots/utils/Motion.hpp>
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <../Data.h>

using namespace std;
using namespace webots;
using namespace chrono;



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
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
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
  Node* robot_node = getFromDef("ball");
  Field* field = robot_node->getField("translation");

  while(step(timeStep) != -1)
  {
    counter++;
    //cout << counter << endl;
    if (counter == 20)
      counter = 0;
      
    now = getTime();
    
    const double* pos = field->getSFVec3f();
    //cout << "position of BALL: " << endl;
    //cout << pos[0] << pos[1] << pos[2] << endl;
    
    Data dataSending(messageID, "main", now, pos[0], pos[1], pos[2]); //!!! just using constatns for velocity right now. please set
    //if (counter == 0)
      cout << "server" << " sending:  (" << dataSending.time << "): " << dataSending.messageID << " " << dataSending.getName() << " " << dataSending.time << " " << dataSending.x << " " << dataSending.y << " " << dataSending.velocityX << " " << dataSending.velocityY << endl;
    //byte* byteArray = new byte[sizeof(dataSending)];
    //memcpy(byteArray, &dataSending, sizeof(dataSending)); //dest, source
    
    emitter->send(&dataSending, sizeof(dataSending));
    messageID++;
    
    if (receiver->getQueueLength()>0){ // will actually have to loop through the queue
      for (int i = 0; i <receiver-> getQueueLength(); i++)
      {
        //int messageSize = receiver->getDataSize();
        //string messageR((const char*)receiver->getData());
        //const byte* byteArray;
        //byteArray = (const byte*) receiver->getData();
        const Data* d = (const Data*) receiver->getData();
        //cout << "byteArray: " << byteArray;
        
        //Data dataReceived;
        //memcpy(&dataReceived, byteArray, messageSize);
        
        now = getTime();
        long ping = (now - d->time);
          
        //if (counter == 0) //print every few steps instead of at each step
          cout << "server" << " received: (" << now << "): " << d->messageID  << " " << d->getName()  << " " << d->time << " " << d->x << " " << d->y << " " << d->velocityX << " " << d->velocityY << "; ping: " << ping << "ms" << endl;
        //cout << counter << endl;
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
