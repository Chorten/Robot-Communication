#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/LED.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/DistanceSensor.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Robot.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Emitter.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Receiver.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/PositionSensor.hpp>
#include </Users/Chorten/Desktop/Webots/Projects/include/controller/cpp/webots/Servo.hpp>

#include <iostream>

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  int timeStep = (int) robot->getBasicTimeStep();
  //LED *led = robot->getLED("ledName");
  DistanceSensor *distanceSensor1 = robot->getDistanceSensor("Sonar/Right");
  DistanceSensor *distanceSensor2 = robot->getDistanceSensor("Sonar/Left");
  //PositionSensor *positionSensor = robot->getPositionSensor("positionSensorName");

  distanceSensor1->enable(timeStep);
  distanceSensor2->enable(timeStep);

  //positionSensor->enable(timeStep);


  char message[128];
  sprintf(message, "hello");
  Emitter(robot->getName()).send(message, sizeof(message) + 1);
  //Motor *move = new Motor(robot->getName());
  //move->setVelocity(100);
  //Keyboard().enable(10);
  //robot-> step(50);
  
  //Main control loop
  while (robot->step(timeStep) != -1) {
    // Read the sensor
    double distanceVal1 = distanceSensor1->getValue();
    double distanceVal2 = distanceSensor2->getValue();

    //double positionVal = positionSensor->getValue();
    cout << distanceVal1 << " "<< distanceVal2 << endl;

    // Process sensor data here

    // Enter here functions to send actuator commands
    //led->set(1);
  }

  delete robot;
  return 0;
}