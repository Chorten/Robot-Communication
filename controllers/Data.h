#include <unistd.h>
#include <cstring>
#include <string>
#include <iostream>

class Data
{
  public:
    size_t messageID;
	static const size_t NAME_SIZE = 4;
	static const size_t MESSAGE_SIZE = 200;
    long time; //time in microseconds
	int role;
    double x;
    double y;
    double z;
    double velocityX;
    double velocityY;
    double velocityZ;
	char message[MESSAGE_SIZE];
	size_t messageSize;
    
    Data(size_t messageID, char* name = NULL, long time = 0, int role = 0, double x = 0, double y = 0, double z = 0, double velocityX = 0, double velocityY = 0, double velocityZ = 0, char* message = "", size_t messageSize = 0);
    void setName(char* name);
	void setMessage(char* message, size_t size);
    std::string getName() const;
	std::string getMessage() const;
  private:
	char name[NAME_SIZE];
    
};
