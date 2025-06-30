/*
 * pid.h
 *
 *  Created on: Apr 18, 2025
 *      Author: Gleb
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#ifndef PID_H
#define PID_H

//#include <cstdint>
#include "stm32f3xx_hal.h"

class PID {
public:
    PID(uint16_t* input, uint16_t* setPoint, double& output,
        double Kp, double Ki, double Kd);

    void compute();

    // Setters
    void setKp(double Kp);
    void setKi(double Ki);
    void setKd(double Kd);
    void setTunings(double Kp, double Ki, double Kd);

    // Getters
    double getKp() const;
    double getKi() const;
    double getKd() const;

private:
    uint16_t* _input;
    uint16_t* _setPoint;
    double& _output;

    double _Kp;
    double _Ki;
    double _Kd;

    double _error;
    double _lastError;
    double _integral;
    double _derivative;
};

#endif // PID_H

#endif /* INC_PID_H_ */
