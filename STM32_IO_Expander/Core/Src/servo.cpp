#include <servo.hpp>

//ServoConfig servos[STM32NumServos];


class AbstractServo {
public:
  virtual void SetAngle(uint16_t Angle);

  uint16_t Angle =0;
};

class PwmServo : public AbstractServo{
public:
	PwmServo(uint16_t pin, uint16_t port);
	void SetAngle(uint16_t Angle);
private:
	uint16_t _pin, _port;
};

class DirectControllServo : public AbstractServo{
public:
	DirectControllServo(uint16_t pin, uint16_t port);
	void SetAngle(uint16_t Angle);
private:
	uint16_t _pin, _port;
};





void PwmServo::SetAngle(uint16_t Angle){


}




extern "C" {
void f(uint16_t dat){

}
};

