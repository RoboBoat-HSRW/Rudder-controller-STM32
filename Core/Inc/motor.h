#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "stm32f3xx_hal.h"  // Or your STM32 HAL header

class Motor {
public:
    Motor(GPIO_TypeDef* port, uint16_t pin,
    	  volatile uint32_t* ccr,
          uint16_t* potValue, uint16_t potMax, uint16_t potMin,
          uint16_t* setPoint,
          double Kp, double Ki, double Kd
          );

    void controlLoop();   // Runs PID and updates CCR
    void setDirection();  // Sets motor direction based on PID output

private:
    GPIO_TypeDef* _myPort;
    uint16_t _myPin;

    uint16_t* _mypotValue;
    uint16_t _mypotMax;
    uint16_t _mypotMin;

    double _myoutput;                      // Output to CCR
    uint16_t* _mysetPoint_CAN_Rx;

    volatile uint32_t* _ccr;                 // Pointer to CCRx register

    PID pid;
};

#endif // MOTOR_H
