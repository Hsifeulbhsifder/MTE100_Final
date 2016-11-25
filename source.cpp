#include "NXT_FileIO.c"
#include "_zm_pid.h"

#define CHORD_MOTOR motorA
#define STRUM_MOTOR motorB
#define PULL_MOTOR motorC

#define STRUM_RANGE (70)
#define PULL_TIME (40)
// kp = 1000
// kp = 100
// kp = 45
#define KPs (1000)
#define KIs (100)
#define KDs (45)
#define E_THRES (2)
#define D_THRES (0.01)

#define KPc (0)
#define KIc (0)
#define KDc (0)

#define DY (10)

#define PID_HZ (360)

#define MAX_SONG_SIZE (1024)

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
	//float vel = 0.15;
	zmPID pid;

	pid.epsilon = 0.01;
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
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

void PIDStrumMiss(float target, float kp, float ki,
			  	  float kd, int speed, float dy){
	float dt = 1.0 / PID_HZ;
	float targetMSPerFrame = 1000.0 * dt;
	float tp = (target<nMotorEncoder[STRUM_MOTOR]?-100:100);
	float vel = speed / 100.0;
	//float vel = 0.05;
	float dx = target - nMotorEncoder[STRUM_MOTOR];
	zmPID pid;

	pid.epsilon = 0.01;
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.target = target;
	pid.dt = dt;
	pid.e = dx;
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
		float alpha = nMotorEncoder[STRUM_MOTOR] / dx;
		if(alpha < 0 || alpha > 1) alpha -= 1;
		float y = motor[PULL_MOTOR];
		float y1 = Clamp(-100, 100, dy *
							(y - 2 * alpha));
		float y2 = 100 * cos(PI * alpha);
		while(y1 == 0){};
		motor[PULL_MOTOR] = (int)(4 * vel * y2) - 200;
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
	motor[PULL_MOTOR] = 0;
}

void PIDChord(float target, float kp, float ki, float kd){
	float dt = 1.0 / PID_HZ;
	float targetMSPerFrame = 1000.0 * dt;
	float tp = (target < 0 ? -100 : 100);
	float dx = target - nMotorEncoder[CHORD_MOTOR];
	zmPID pid;

	pid.epsilon = 0.01;
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.target = target;
	pid.dt = dt;
	pid.e = dx;
	pid.i = 0;
	pid.d = pid.e / pid.dt;
	nxtDisplayString(3, "%de+%di+%dd", pid.kp, pid.ki, pid.kd);

	time1[T3] = 0;
	float lastTime = time1[T3];
	while(Abs(pid.e) > E_THRES || Abs(pid.d) > D_THRES){
		float u = ClosedPIDStep(pid,
				  nMotorEncoder[CHORD_MOTOR]);
		motor[CHORD_MOTOR] = Clamp(-100, 100, tp + u);
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
	motor[CHORD_MOTOR] = 0;
	
}

void Test(int speed){
	if(speed > 10){
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed, DY);
	}
}

typedef struct{
	char chordData;
	char strumData;
} SongFrame;

typedef struct{
	int size;
	char tempo;
	char frames[MAX_SONG_SIZE];
} SongData;

void Callibrate(){
	//TODO: Calibrate by using sensors
	while(!SensorValue(S2)) motor[STRUM_MOTOR] = -50;
	motor[STRUM_MOTOR] = 0;
	nMotorEncoder[PULL_MOTOR] = 0;
	nMotorEncoder[STRUM_MOTOR] = 0;
}

void GetFrameData(char byte, int& chord, int& strum){
	chord = (int)((byte >> 5) & 0x07);
	strum = 4 * (int)((byte & 0x1F));
}

void GetSongData(TFileHandle fin, SongData &data){
	data.size = 0;
	int tempo;
	readIntPC(fin, tempo);
	data.tempo = (char) tempo;
	int temp = 0;
	while(readIntPC(fin, temp)){
		data.frames[data.size] =  (char)((0b00000111 &
								  (char)temp) << 5);
		readIntPC(fin, temp);
		data.frames[data.size] += (char)((0b00011111 &
								  (char)(temp / 4)));
		data.size++;
	}
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

static SongData songData; //NOTE: This is global because there
						  //      is not enough local stack
						  //      space to house a buffer this
						  //      large.

task main(){
	SensorType[S2] = sensorTouch;
	bMotorReflected[motorB] = true;
	bMotorReflected[motorC] = true;
	Callibrate();
	wait1Msec(1000);
	while(1) Test(100);
	TFileHandle fin;
	if(openReadPC(fin, "songData.txt")){
		nxtDisplayString(5, "We Good");
		int strum, chord;
		GetSongData(fin, songData);
		float targetMSPerFrame = 60000.0 / 20.0;
		time1[T1] = 0;
		float lastTime = time1[T1];
		for(int i = 0; i < songData.size; i++){
			GetFrameData(songData.frames[i], chord, strum);

			//Process Input
			//HoldChord(frame.chordData, lastFrame.chordData);

			//Strum(frame.strumData);
			Test(strum);

			//Time Processing
			ProcessTime(lastTime, targetMSPerFrame, T1);

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
