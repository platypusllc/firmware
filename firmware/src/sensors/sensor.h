#ifndef SENSOR_H
#define SENSOR_H

class Sensor
{
 protected:
  Sensor() { };
  virtual ~Sensor() { };
  
 public:
  virtual void loop(void) = 0;
  virtual void update(void) = 0;
};

#endif /* SENSOR_H */
