#include "Motor.h"

Motor::Motor(GPIO_TypeDef* port, uint16_t pin,
			 volatile uint32_t* ccr,
             uint16_t* potValue, uint16_t potMax, uint16_t potMin,
             uint16_t* setPoint,
             double Kp, double Ki, double Kd)
    : _myPort(port),
      _myPin(pin),
      _mypotValue(potValue),
      _mypotMax(potMax),
      _mypotMin(potMin),
      _myoutput(0),
      _mysetPoint_CAN_Rx(setPoint),
      _ccr(ccr),
      pid(_mypotValue, _mysetPoint_CAN_Rx, _myoutput, Kp, Ki, Kd)
{
}

void Motor::setDirection() {
    HAL_GPIO_WritePin(_myPort, _myPin, (_myoutput > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor::controlLoop() {
    setDirection();
    pid.compute();
    if ((_myoutput > 0 && *_mypotValue < _mypotMax) ||
    	    (_myoutput < 0 && *_mypotValue > _mypotMin))
    {
    	*_ccr = static_cast<uint32_t>((_myoutput < 0) ? -_myoutput : _myoutput);
    } else {
    	*_ccr = 0;
    }
}

