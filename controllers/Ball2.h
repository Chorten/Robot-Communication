#include <unistd.h>
#include <cstring>
#include <string>
#include <iostream>

using namespace std;

class Ball2
    {
        private:
          double ball_x;
          double ball_z;
          string id;
        public:
          double getx(){return ball_x;}
          double getz(){return ball_z;}
          void setPos(double x, double z)
          {
            ball_x = x;
            ball_z = z;
          }
          string getid()
          {
            id = "ball";
            return id;
          }
    };