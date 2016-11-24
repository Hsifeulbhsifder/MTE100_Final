#ifndef _ZM_PID_H
#define _ZM_PID_H

typedef struct{
	float epsilon;
	float kp;
	float ki;
	float kd;
	float target;
	float dt;
	float e;
	float i;
	float d;
} zmPID;

float Abs(float x){
	return (x < 0 ? -x : x);
}

float Clamp(float min, float max, float x){
	return (x < min ? min : (x > max ? max : x));
}

float ClosedPIDStep(zmPID& pid, float obs){
	float e = pid.target - obs;
	if(Abs(e) > pid.epsilon){
		pid.i += e * pid.dt;
	}
	pid.d = (e - pid.e) / pid.dt;
	float u = pid.kp * e + pid.ki * pid.i + pid.kd * pid.d;

	pid.e = e;
	return u / 100.0;
}

#endif
