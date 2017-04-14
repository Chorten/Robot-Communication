#include <../Data.h>

Data::Data(size_t messageID, char* name, long time, double x, double y, double z, double velocityX, double velocityY, double velocityZ, char* message, size_t messageSize)
{
  this->messageID = messageID;
  setName(name); // new char[4];
  //memcpy(this->name, name, 4);
  this->time = time; //time in microseconds
  this->x = x;
  this->y = y;
  this->z = z;
  this->velocityX = velocityX;
  this->velocityY = velocityY;
  this->velocityZ = velocityZ;
  setMessage(message, messageSize);
}

void Data::setName(char* name)
{
  memcpy(this->name, name, NAME_SIZE);
}

std::string Data::getName() const
{
  return std::string(name, NAME_SIZE);
}

void Data::setMessage(char* message, size_t size)
{
  messageSize = (size > MESSAGE_SIZE ? MESSAGE_SIZE : size);
  memcpy(this->message, message, messageSize);
}

std::string Data::getMessage() const
{
  return std::string(message, messageSize);
}
