#include "NXT_FileIO.c"
#include "_zm_pid.h"

#define CHORD_MOTOR motorA
#define STRUM_MOTOR motorB
#define PULL_MOTOR motorC

#define STRUM_RANGE (63)
#define PULL_TIME (40)
// kp = 1000
// kp = 100
// kp = 45
#define KPs (1000)
#define KIs (100)
#define KDs (45)
#define E_THRES (5)
#define D_THRES (0.1)

#define KPc (1750)
#define KIc (120)
#define KDc (65)

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
		float x = nMotorEncoder[STRUM_MOTOR];
		float u = ClosedPIDStep(pid, x);
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
		float x = nMotorEncoder[STRUM_MOTOR];
		float u = ClosedPIDStep(pid, x);
		motor[STRUM_MOTOR] = vel * Clamp(-100, 100, tp + u);
		float alpha = x / dx;
		if(alpha < 0 || alpha > 1) alpha -= 1;
		/*float y = motor[PULL_MOTOR];
		float y1 = Clamp(-100, 100, dy *
							(y - 2 * alpha));*/
		float y2 = 100 * cos(PI * alpha);
		motor[PULL_MOTOR] = 1.25 * ((int)(y2) - 30);
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
	nMotorEncoder[CHORD_MOTOR] = 0;
	float dt = 1.0 / PID_HZ;
	float targetMSPerFrame = 1000.0 * dt;
	float dx = target - 0;
	float tp = (dx < 0 ? -100 : 100);
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
		float x = nMotorEncoder[CHORD_MOTOR] % 360;
		float u = ClosedPIDStep(pid, x);
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

void Temp(int speed){
	for(int i = 0; i < 30; i++){
		PIDChord(90 * i, KPc, KIc, KDc);
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrum(0, KPs, KIs, KDs, speed);
		wait1Msec(500);
	}
}

void Test(int speed){
	if(speed > 10){
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		//PIDStrum(0, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed * 0.25, DY);
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
	while(!SensorValue(S2)) motor[STRUM_MOTOR] = -15;
	motor[STRUM_MOTOR] = 0;
	motor[PULL_MOTOR] = -15;
	wait1Msec(200);
	motor[PULL_MOTOR] = 0;
	nMotorEncoder[PULL_MOTOR] = 0;
	nMotorEncoder[STRUM_MOTOR] = 0;
	nMotorEncoder[CHORD_MOTOR] = 0;
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
	if(chordData == 0 && lastChordData == 3){
		int x = 1;
	}
	int modChord = (chordData - lastChordData) % 4 - 2;
	if(modChord == 1){
		PIDChord(90, KPc, KIc, KDc);
	} else if(modChord == -1 || modChord == -5){
		PIDChord(-90, KPc, KIc, KDc);
	} else if(modChord == 0){
		PIDChord(180, KPc, KIc, KDc);
	}
}

void Strum(int speed){
	if(speed > 10){
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		//PIDStrum(0, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed * 0.25, DY);
	}
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
	//while(1) Test(100);
	TFileHandle fin;
	if(openReadPC(fin, "4chords.txt")){
		nxtDisplayString(5, "We Good");
		int strum, chord, lastChord = 0;
		GetSongData(fin, songData);
		float targetMSPerFrame = 60000.0 / (songData.tempo);
		while(1){
			if(nNxtButtonPressed == 1){
				PIDChord(-90, KPc, KIc, KDc);
				Strum(100);
			}else if(nNxtButtonPressed == 2){
				PIDChord(90, KPc, KIc, KDc);
				Strum(100);
			}else if(nNxtButtonPressed == 3){

				time1[T1] = 0;
				float lastTime = time1[T1];
				for(int i = 0; i < songData.size; i++){
					GetFrameData(songData.frames[i],
								 chord, strum);

					//Process Input
					HoldChord(chord, lastChord);
					lastChord = chord;
					//Strum(frame.strumData);
					for(int j = 0; j < 1; j++) Strum(strum);

					//Time Processing
					ProcessTime(lastTime, targetMSPerFrame, T1);

					float endTime = time1[T1];
					float frameTimeMS = endTime - lastTime;
					lastTime = endTime;
					float bpm = 60000.0 / frameTimeMS;

					//Frame logging
					nxtDisplayString(7, "%.02f(%.02f)",
									 frameTimeMS
									 / 1000.0, bpm);
				}
			}
		}

	}else{
		nxtDisplayString(0, "Failure");
	}

}
