#include <distance.h>


DistanceSensor::DistanceSensor(void) : 
interface{nullptr}, data_buffer{0}, cs{0}
{
}

DistanceSensor::~DistanceSensor()
{
}

void DistanceSensor::setup_sensor(Stream &serialPort){
    if(!interface){
        interface = &serialPort;
    }

    // Flush all of the bytes before initialization
    while (interface->available())
    {
        interface->read();
    }   
}

int DistanceSensor::get_distance(){
  if (interface->available() > 0){
    sleep(4);

    if (interface->read() == 0xff)
    {
      data_buffer[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = interface->read();
      }

      cs = data_buffer[0] + data_buffer[1] + data_buffer[2];

      if (data_buffer[3] == cs) {
        return (data_buffer[1] << 8) + data_buffer[2];
      }
      else{
        return 0;
      }
    }
    else{
      return 0;
    }
  }
  else{
    return 0;
  }
}