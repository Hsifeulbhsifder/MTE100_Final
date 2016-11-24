#include "NXT_FileIO.c"
#include "_zm_pid.h"

#define CHORD_MOTOR motorA
#define STRUM_MOTOR motorB
#define PULL_MOTOR motorC

#define STRUM_RANGE (65)
#define PULL_TIME (40)
// kp = 1000
// kp = 100
// kp = 45
#define KP (1000)
#define KI (100)
#define KD (45)
#define E_THRES (2)
#define D_THRES (0.1)

#define PID_HZ (360)


void ProcessTime(float lastTime, float targetMSPerFrame,
				 int timer){
	float elapsedMS = time1[timer] - lastTime;
	if(elapsedMS < targetMSPerFrame) {
		wait1Msec(targetMSPerFrame - elapsedMS);
		float testElapsedMS = time1[timer] - lastTime;

		if(testElapsedMS > targetMSPerFrame) {
			//nxtDisplayString(0, "Long Wait");
		} else {
			do {
				elapsedMS = time1[timer] - lastTime;
			} while(elapsedMS <= targetMSPerFrame);
		}

	} else {
		nxtDisplayString(0, "Missed Frame");
	}
}

void PIDStrum(float target, float kp, float ki,
			  float kd, int speed){
	float dt = 1.0 / PID_HZ;
	float targetMSPerFrame = 1000.0 * dt;
	float tp = (target<nMotorEncoder[STRUM_MOTOR]?-100:100);
	float vel = speed / 100.0;
	zmPID pid;

	pid.epsilon = 0.01;
	pid.kp = KP;
	pid.ki = KI;
	pid.kd = KD;
	pid.target = target;
	pid.dt = dt;
	pid.e = pid.target - nMotorEncoder[STRUM_MOTOR];
	pid.i = 0;
	pid.d = pid.e / pid.dt;
	bool zeroed = false;
	nxtDisplayString(3, "%de+%di+%dd", pid.kp, pid.ki, pid.kd);
	time1[T3] = 0;
	float lastTime = time1[T3];
	while(Abs(pid.e) > E_THRES || Abs(pid.d) > D_THRES
		  && !zeroed){
		if(SensorValue(S2)){
			zeroed = true;
			nMotorEncoder[STRUM_MOTOR] = 0;
		}
		float u = ClosedPIDStep(pid,
				  nMotorEncoder[STRUM_MOTOR]);
		motor[STRUM_MOTOR] = vel * Clamp(-100, 100, tp + u);
		nxtDisplayString(1, "%d", tp + u);
		nxtDisplayString(1, "%d", tp + u);

		//Time Processing
		ProcessTime(lastTime, targetMSPerFrame, T3);

		float endTime = time1[T3];
		float frameTimeMS = endTime - lastTime;
		lastTime = endTime;
		float hz = 1000.0 / frameTimeMS;

		//Frame logging
		nxtDisplayString(6, "%.04f(%.02f)",
						 frameTimeMS / 1000.0, hz);
	}
	motor[STRUM_MOTOR] = 0;
}

void Test(int speed){
	if(speed > 10){
		PIDStrum(STRUM_RANGE, KP, KI, KD, speed);
		PIDStrum(0, KP, KI, KD, speed);
	}
}

typedef struct{
	int chordData;
	int strumData;
} SongFrame;

void Callibrate(){
	//TODO: Calibrate by using sensors
	while(!SensorValue(S2)) motor[STRUM_MOTOR] = -50;
	motor[STRUM_MOTOR] = 0;
	nMotorEncoder[PULL_MOTOR] = 0;
	nMotorEncoder[STRUM_MOTOR] = 0;
}

bool GetSongFrame(TFileHandle fin, SongFrame &frame){
	bool read = true;
	read &= readIntPC(fin, frame.chordData);
	read &= readIntPC(fin, frame.strumData);
	return read;
}

void Turn90Degrees(int direction, int speed){
	nMotorEncoder[CHORD_MOTOR] = 0;
	motor[CHORD_MOTOR] = speed * direction;
	while(nMotorEncoder[CHORD_MOTOR] < 90 * direction){}
	motor[CHORD_MOTOR] = 0;
}

void Turn180Degrees(int speed){
	nMotorEncoder[CHORD_MOTOR] = 0;
	motor[CHORD_MOTOR] = speed;
	while(nMotorEncoder[CHORD_MOTOR] < 180){}
	motor[CHORD_MOTOR] = 0;
}

void HoldChord(int chordData, int lastChordData){
	int modChord = (chordData - lastChordData) % 4 - 2;
	if(modChord == -1 || modChord == 1){
		Turn90Degrees(modChord, 100);
	}
	else if(modChord == 0){
		Turn180Degrees(100);
	}
}

void Strum(int speed){
	nMotorEncoder[STRUM_MOTOR] = 0;
	motor[STRUM_MOTOR] = 100;
	while(nMotorEncoder[STRUM_MOTOR] < STRUM_RANGE){}
	motor[STRUM_MOTOR] = 0;
	nMotorEncoder[PULL_MOTOR] = 0;
	motor[PULL_MOTOR] = 100;
	/*while(nMotorEncoder[PULL_MOTOR] < PULL_RANGE){
		nxtDisplayString(4, "%d", nMotorEncoder[PULL_MOTOR]);
	}*/
	wait1Msec(PULL_TIME);
	motor[PULL_MOTOR] = 0;
	motor[STRUM_MOTOR] = -10;
	while(nMotorEncoder[STRUM_MOTOR] > 0){}
	motor[STRUM_MOTOR] = 10;
	while(nMotorEncoder[STRUM_MOTOR] < 0){}
	motor[STRUM_MOTOR] = 0;
	motor[PULL_MOTOR] = -100;
	/*while(nMotorEncoder[PULL_MOTOR] > 0){}*/
	wait1Msec(PULL_TIME);
	motor[PULL_MOTOR] = 0;
}

task main(){
	SensorType[S2] = sensorTouch;
	bMotorReflected[motorB] = true;
	bMotorReflected[motorC] = true;
	Callibrate();
	wait1Msec(1000);
	TFileHandle fin;
	if(openReadPC(fin, "songData.txt")){
		nxtDisplayString(5, "We Good");
		int tempo;
		SongFrame frame;
		SongFrame lastFrame;
		readIntPC(fin, tempo);
		int iterator = 0;
		float targetMSPerFrame = 60000.0 / 140.0;
		time1[T1] = 0;
		float lastTime = time1[T1];
		while(GetSongFrame(fin, frame)){
			//Process Input
			//HoldChord(frame.chordData, lastFrame.chordData);

			//Strum(frame.strumData);
			Test(frame.strumData);

			//Time Processing
			ProcessTime(lastTime, targetMSPerFrame, T1);

			lastFrame = frame;

			float endTime = time1[T1];
			float frameTimeMS = endTime - lastTime;
			lastTime = endTime;
			float bpm = 60000.0 / frameTimeMS;

			//Frame logging
			nxtDisplayString(7, "%.02f(%.02f)", frameTimeMS
							 / 1000.0, bpm);
		}

	}else{
		nxtDisplayString(0, "Failure");
	}

}
