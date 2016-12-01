#include "NXT_FileIO.c"

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
#define KIc (480)
#define KDc (8)

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
	float sign = 1;
	if(target > nMotorEncoder[STRUM_MOTOR]){
		sign = -1;
	}
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
		motor[PULL_MOTOR] = sign * 1.25 * ((int)(y2) - 30);
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
	chord = (int)((byte >> 6) & 0x03);
	strum = 2 * (int)((byte & 0x3F));
}

void GetSongData(TFileHandle fin, SongData &data){
	data.size = 0;
	int tempo;
	readIntPC(fin, tempo);
	data.tempo = (char) tempo;
	int temp = 0;
	while(readIntPC(fin, temp)){
		data.frames[data.size] =  (char)((0x03 &
								  (char)temp) << 6);
		readIntPC(fin, temp);
		data.frames[data.size] += (char)((0x3F &
								  (char)(temp / 2)));
		data.size++;
	}
}

void HoldChord(int chordData, int lastChordData){
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
		#if 1
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed * 0.25, DY);
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrum(0, KPs, KIs, KDs, speed);
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed * 0.25, DY);
		#else
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed * 0.25, DY);
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrum(0, KPs, KIs, KDs, speed);
		PIDStrumMiss(STRUM_RANGE, KPs, KIs, KDs,
					 speed * 0.25, DY);
		PIDStrum(0, KPs, KIs, KDs, speed);
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrum(0, KPs, KIs, KDs, speed);
		#endif
	}
}

#if 0
void TempSong(){
	while(1){
		for(int i = 0; i < 4; i++){
			Strum(100);
		}
		PIDChord(-90, KPc, KIc, KDc);
		for(int i = 0; i < 4; i++){
			Strum(100);
		}
		PIDChord(90, KPc, KIc, KDc);
	}
}
#endif

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
	//TempSong();
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
					Strum(strum);

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
