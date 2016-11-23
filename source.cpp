#include "NXT_FileIO.c"

#define CHORD_MOTOR motorA
#define STRUM_MOTOR motorB
#define PULL_MOTOR motorC

#define STRUM_RANGE 45
#define PULL_RANGE 60

typedef struct{
	int chordData;
	int strumData;
} SongFrame;

void Callibrate(){
	//TODO: Calibrate by using sensors
	motor[PULL_MOTOR] = -30;
	wait1Msec(600);
	motor[PULL_MOTOR] = 0;
	SensorType[S2] = sensorTouch;
	motor[STRUM_MOTOR] = -50;
	while(!SensorValue(S2)){}
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
	motor[STRUM_MOTOR] = speed;
	while(nMotorEncoder[STRUM_MOTOR] < STRUM_RANGE){}
	motor[STRUM_MOTOR] = 0;
	nMotorEncoder[PULL_MOTOR] = 0;
	motor[PULL_MOTOR] = 100;
	while(nMotorEncoder[PULL_MOTOR] < PULL_RANGE){}
	motor[PULL_MOTOR] = 0;
	nMotorEncoder[STRUM_MOTOR] = 0;
	motor[STRUM_MOTOR] = -100;
	while(nMotorEncoder[STRUM_MOTOR] > 0){}
	motor[STRUM_MOTOR] = 0;
	nMotorEncoder[PULL_MOTOR] = 0;
	motor[PULL_MOTOR] = 100;
	while(nMotorEncoder[PULL_MOTOR] > 0){}
	motor[PULL_MOTOR] = 0;
}

void ProcessTime(float lastTime, float targetMSPerFrame){
	float elapsedMS = time1[T1] - lastTime;
	if(elapsedMS < targetMSPerFrame) {
		wait1Msec(targetMSPerFrame - elapsedMS);
		float testElapsedMS = time1[T1] - lastTime;

		if(testElapsedMS > targetMSPerFrame) {
			//nxtDisplayString(0, "Long Wait");
		} else {
			do {
				elapsedMS = time1[T1] - lastTime;
			} while(elapsedMS <= targetMSPerFrame);
		}

	} else {
		nxtDisplayString(0, "Missed Frame");
	}
}

task main(){
	Callibrate();

	TFileHandle fin;
	if(openReadPC(fin, "songData.txt")){

		int tempo;
		SongFrame frame;
		SongFrame lastFrame;
		readIntPC(fin, tempo);

		float targetMSPerFrame = 60000.0 / tempo;
		time1[T1] = 0;
		float lastTime = time1[T1];
		while(GetSongFrame(fin, frame)){
			//Process Input
			HoldChord(frame.chordData, lastFrame.chordData);

			Strum(frame.strumData);

			//Time Processing
			ProcessTime(lastTime, targetMSPerFrame);

			lastFrame = frame;

			float endTime = time1[T1];
			float frameTimeMS = endTime - lastTime;
			lastTime = endTime;
			float hz = 1000.0 / frameTimeMS;

			//Frame logging
			nxtDisplayString(1, "%.02fms/f (%.02fHz)",
							 frameTimeMS, hz);
		}

	}else{
		nxtDisplayString(0, "Failure");
	}

}
