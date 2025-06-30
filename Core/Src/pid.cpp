#include "PID.h"

// Constructor
PID::PID(uint16_t* input, uint16_t* setPoint, double& output,
         double Kp, double Ki, double Kd)
    : _input(input),
      _setPoint(setPoint),
      _output(output),
      _Kp(Kp), _Ki(Ki), _Kd(Kd),
      _error(0.0), _lastError(0.0), _integral(0.0), _derivative(0.0)
{ }

// Main PID compute function
void PID::compute() {
    if (!_input || !_setPoint)
        return;

    _error = static_cast<double>(*_setPoint) - static_cast<double>(*_input);
    _integral += _error;
    _derivative = _error - _lastError;

    _output = (_Kp * _error) + (_Ki * _integral) + (_Kd * _derivative);

    /*// Clamp to 16-bit range
    if (output > 65535.0) output = 65535.0;
    else if (output < 0.0) output = 0.0;

    _output = output;  // ✔️ No dereference needed */

    _lastError = _error;
}

// Setters
void PID::setKp(double Kp) { _Kp = Kp; }
void PID::setKi(double Ki) { _Ki = Ki; }
void PID::setKd(double Kd) { _Kd = Kd; }
void PID::setTunings(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

// Getters
double PID::getKp() const { return _Kp; }
double PID::getKi() const { return _Ki; }
double PID::getKd() const { return _Kd; }
