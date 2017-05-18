
#include <webots/utils/Motion.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Device.hpp>
#include <unistd.h>
#include <cmath>

using namespace webots;
using namespace std;

#define NE "NE"
#define SE "SE"
#define NW "NW"
#define SW "SW"
#define PI 3.14159265

string filename1 = "../motions/HandWave.motion"; 
string filename2 = "../motions/Forwards.motion"; 
string right60 = "../motions/TurnRight60.motion"; 
string right40 = "../motions/TurnRight40.motion"; 
string left40 = "../motions/TurnLeft40.motion"; 
string left60 = "../motions/TurnLeft60.motion"; 
string left180 = "../motions/TurnLeft180.motion";

class TestNao : public Robot
{
  public:
    TestNao(char* name);
    void run();
    char* getName();
    double getAngle(double ball_x, double ball_y);
    void rotate(double angle, string myDirect, string ballDirect);
    void move(string filename);
    void setMotion(double angle, string ballDirect);
    string getDirection(const double* loc);

  private:
    char* name;
    int timeStep;
    GPS* gps;
    string myDirection;
    string BallDirection;
    double myPosX;
    double myPosY;   
};

TestNao::TestNao(char* name)
{
  this->name = name;
  timeStep = 16;
  gps = new GPS("gps");
  gps->enable(100);
  myDirection = "NORTH";
  BallDirection = NE;
  //IU = new InertialUnit("inertialunit");
  //IU->enable(100);
}

void TestNao::run()
{
    const double* loc = gps->getValues();
    myPosX = loc[2];
    myPosY = loc[0];
    double myangle = getAngle(2.0, 2.0);
    //cout<<"Direction: "<< getDirection(gps->getValues())<<endl;
    rotate(myangle, this->myDirection, this->BallDirection);
}


double TestNao::getAngle(double ball_x, double ball_y)
{
  return abs(atan((ball_x - myPosX)/(ball_y - myPosY))*180 / PI);
}


void TestNao::rotate(double angle, string myDirect, string ballDirect)
{
  cout<<"Angle: "<<angle<<" My direction: "<<myDirect<<endl;
  if(myDirect == "NORTH")
  {
    if(ballDirect == NE || ballDirect == NW)
    {
      setMotion(angle, ballDirect);
      this->myDirection = "NORTH";
    }
    else if(ballDirect == SE || ballDirect == SW)
    {
      angle = 180 - angle;
      setMotion(angle, ballDirect);
      this->myDirection = "SOUTH";
    }
  }
  else if(myDirect == "SOUTH")
  {
    if(ballDirect == NE || ballDirect == NW)
    {
      setMotion(angle, ballDirect);
      this->myDirection = "NORTH";
    } 
    else if(ballDirect == SE || ballDirect == SW)
    {
      angle = 180 - angle;
      setMotion(angle, ballDirect);
      this->myDirection = "SOUTH";
    }
    
  }
}

void TestNao::setMotion(double angle, string ballDirect)
{
  if(angle< 30) move(filename2);
  else if(angle>=30 && angle <50)
  {
    if(ballDirect == NE||ballDirect == NW)
    {
      move(right40);
      move(filename2);
    }
    else if(ballDirect == SE||ballDirect == SW) 
    {
      move(left40);
      move(filename2);
    }
  }
  else if(angle>=50 && angle <70)
  {
    if(ballDirect == NE || ballDirect == NW)
    {
      move(right60);
      move(filename2);
    }
    else if(ballDirect == SE || ballDirect == SW)
    {
      move(left60);
      move(filename2);
    }
  }
  else if(angle >= 70)
  {
    if(ballDirect == NE || ballDirect == NW) 
    {
      move(right60);
      angle-=60;
      setMotion(angle,ballDirect);
    }
    else if(ballDirect == SE || ballDirect == SW) 
    {
      move(left60);
      angle-=60;
      setMotion(angle,ballDirect);
    }
  }
  
}


void TestNao::move(string filename)
{
  Motion *movement = new Motion(filename);
  if (! movement->isValid())
  {
    cout << "could not load file: " << filename << std::endl;
    delete movement;
  }
  movement->setLoop(true);
  movement->play();
  movement->setLoop(false);
  
}

//Ball direction
string TestNao::getDirection(double x, double y)
{
  //double x= loc[2];
  //double y = loc[0];
  if(x>0.0 && x<3.0)
  {
    if(y>0.0 && y<4.5) return NE;
    else if (y<0.0 && y > -4.5) return SE;
  }
  else if(x<0.0 && x>-3.0)
  {
    if(y>0.0 && y<4.5) return NW;
    else if (y<0.0 && y > -4.5) return SW;
  }
  else return "Out of Bound";
}

int main(int argc, char **argv)
{
    TestNao Nao(argv[1]);
    Nao.run();
    return 0;
}