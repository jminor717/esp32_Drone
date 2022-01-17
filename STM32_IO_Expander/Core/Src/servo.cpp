#include <servo.hpp>
#include "tim.h"
#include <cstring>
#include <stdlib.h>
// ServoConfig servos[STM32NumServos];

class AbstractServo
{
public:
    int16_t Angle = 0;
    virtual void SetAngle(int16_t Angle) = 0;
    virtual void Init() = 0;
    AbstractServo(int16_t i) { Angle = i; }
};

class DirectControllServo : public AbstractServo
{
    TIM_HandleTypeDef MP_Tim;
    uint32_t MP_Chan;
    volatile uint32_t *MP_CCR;
    TIM_HandleTypeDef MN_Tim;
    uint32_t MN_Chan;
    volatile uint32_t *MN_CCR;

public:
    DirectControllServo(
        TIM_HandleTypeDef _MP_Tim, uint32_t _MP_Chan, volatile uint32_t *_MP_CCR,
        TIM_HandleTypeDef _MN_Tim, uint32_t _MN_Chan, volatile uint32_t *_MN_CCR) : AbstractServo(0)
    {
        MP_Tim = _MP_Tim;
        MP_Chan = _MP_Chan;
        MP_CCR = _MP_CCR;
        MN_Tim = _MN_Tim;
        MN_Chan = _MN_Chan;
        MN_CCR = _MN_CCR;
    }
    void SetAngle(int16_t Angle) {
//    	*MP_CCR = Angle;
//    	*MN_CCR = Angle;
    	//TIM1->CCR1 = Angle;
    	if(Angle > 0){
    		*MN_CCR = Angle;
    		*MP_CCR = 0;
    	}
    	else if (Angle < 0)
    	{
    		*MP_CCR = -Angle;
    		*MN_CCR = 0;
    	}
    	else{
    		*MP_CCR = 0;
    		*MN_CCR = 0;
    	}
    }
    void Init()
    {
        HAL_TIM_PWM_Start(&MP_Tim, MP_Chan);
        HAL_TIM_PWM_Start(&MN_Tim, MN_Chan);
    }
};

// class PwmServo : public AbstractServo{
// public:
//	PwmServo(uint16_t pin, uint16_t port);
//	void SetAngle(uint16_t Angle);
// private:
//	uint16_t _pin, _port;
// };
// void PwmServo::SetAngle(uint16_t Angle){}
//
// class DirectControllServo : public AbstractServo{
// public:
//	DirectControllServo();
//	~DirectControllServo();
//		//AbstractServo();

//	void SetAngle(uint16_t Angle);
// private:
//	uint16_t _pin, _port;
//
//};

AbstractServo *M1 = new DirectControllServo(
    htim1, TIM_CHANNEL_2, &TIM1->CCR2,
    htim1, TIM_CHANNEL_3, &TIM1->CCR3);

AbstractServo *M2 = new DirectControllServo(
    htim2, TIM_CHANNEL_1, &TIM2->CCR1,
    htim1, TIM_CHANNEL_1, &TIM1->CCR1);

AbstractServo *M3 = new DirectControllServo(
    htim3, TIM_CHANNEL_3, &TIM3->CCR3,
    htim4, TIM_CHANNEL_1, &TIM4->CCR1);

AbstractServo *M4 = new DirectControllServo(
    htim4, TIM_CHANNEL_3, &TIM4->CCR3,
    htim4, TIM_CHANNEL_2, &TIM4->CCR2);

AbstractServo *Servos[STM32NumServos] = {M1, M2, M3, M4};

extern "C"
{
    void ServosInit()
    {
    	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
         HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
         for (size_t i = 0; i < STM32NumServos; i++)
         {
             Servos[i]->Init();
        }
    }

    void SetServoAngle(int16_t angle, uint16_t servoNum)
    {
    	Servos[servoNum]->SetAngle(angle);
    }

};
