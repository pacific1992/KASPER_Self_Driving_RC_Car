#ifndef L5_APPLICATION_MOTOR_HPP_
#define L5_APPLICATION_MOTOR_HPP_
#include "io.hpp"
#include "task.h"
#include "utilities.h"
#include "lpc_pwm.hpp"
#include <stdio.h>


extern int s_left;
extern int s_center;
extern int s_right;
extern float distance;
extern float heading;
extern float bearing;

extern int white_value,max_light_value,min_light_value,temp_count,rpm;



class MotorController : public SingletonTemplate<MotorController>
{
public:
	void setDC(float v);
	void setServo(float v);
private:
	PWM driveMotor;
	PWM steerMotor;
	MotorController();
	friend class SingletonTemplate<MotorController>;
};
#define MotorControl MotorController::getInstance()
#ifdef __cplusplus
extern "C"{
#endif
void servo_left(void);
void servo_right(void);
void servo_straight(void);
void dc_accelerate(float pwmValue);
void dc_stop(void);
void drive_car(int count);
void handle_motor_mia(void);
bool rpm_sensor(void);
void dcmotor_init(void);
void rx_rpm(void);
//void drive_motor();
void drive_with_feedback(void);
void receive_motor_commands(void);

#ifdef __cplusplus
}
#endif
#endif /* L5_APPLICATION_MOTOR_HPP_ */
