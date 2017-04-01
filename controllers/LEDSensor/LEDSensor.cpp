#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <iostream>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  int timeStep = (int) robot->getBasicTimeStep();
  LED *led = robot->getLED("ledName");
  DistanceSensor *distanceSensor = robot->getDistanceSensor("distanceSensorName");
  PositionSensor *positionSensor = robot->getPositionSensor("positionSensorName");
  
  distanceSensor->enable(timeStep);
  positionSensor->enable(timeStep);


  char message[128];
  sprintf(message, "hello");
  Emitter(robot->getName()).send(message, sizeof(message) + 1);
  //Motor *move = new Motor(robot->getName());
  //move->setVelocity(100);
  //Keyboard().enable(10);
  //robot-> step(50);
  
  //Main control loop
  while (robot->step(timeStep) != -1) {
    // Read the sensors
    double distanceVal = distanceSensor->getValue();
    double positionVal = positionSensor->getValue();
    cout << distanceVal << " " << positionVal << endl;

    // Process sensor data here

    // Enter here functions to send actuator commands
    led->set(1);
  }

  delete robot;
  return 0;
}