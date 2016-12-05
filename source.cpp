#include "NXT_FileIO.c"

// Using aliases to determine which motor does what
#define CHORD_MOTOR motorA
#define STRUM_MOTOR motorB
#define PULL_MOTOR motorC

// Angle the strumming motor must turn to complete one strum
#define STRUM_RANGE (63)

// The PID Coefficients for the strumming motor
#define KPs (1000)
#define KIs (100)
#define KDs (45)

// The error threshold to be considered in the correct position
// for the PID control loop
#define E_THRES (5)

// The derivative threshold to be confirm that the target
// position has been reached without oscillation
#define D_THRES (0.1)

// The PID coefficients for the chord motor
#define KPc (1750)
#define KIc (480)
#define KDc (8)

// The coefficient that dictates how high the pulling motor must
// travel in order to miss the strings
#define DY (1.25)

// The frequency of which the PID loop is updated
#define PID_HZ (360)

// The maximum amount of chord-strum pairing that can be stored
// in RAM for the robot to play
#define MAX_SONG_SIZE (1024)

/** Author: Ben Krowchuk
 *  This function takes a target time per loop, as well as the last
 *  time that recorded on the given timer. It calculates how much
 *  time to wait for in order to have uniform  loop times.
 */
void ProcessTime(float lastTime, float targetMSPerFrame,
				 int timer){
	float elapsedMS = time1[timer] - lastTime;
	if(elapsedMS < targetMSPerFrame) {
		wait1Msec(targetMSPerFrame - elapsedMS);
		float testElapsedMS = time1[timer] - lastTime;

		if(testElapsedMS > targetMSPerFrame) {
			nxtDisplayString(0, "Long Wait");
		} else {
			do {
				elapsedMS = time1[timer] - lastTime;
			} while(elapsedMS <= targetMSPerFrame);
		}

	} else {
		nxtDisplayString(0, "Missed Frame");
	}
}

/** Author: Zaeem Mohamed
 *  A struct that encompasses the necessary data to be used in a PID
 *  control loop.
 */
typedef struct{
	float kp; // coefficient for the proportional term
	float ki; // coefficient for the integral term
	float kd; // coefficient for the derivative term
	float target; // target set-point value
	float dt; // delta time (PID loop time)
	float e; // error (target - observed value)
	float i; // accumulative error (sum of errors)
	float d; // derivative of error (difference in error over time)
} zmPID;

/** Author: Taylor Robertson
 *  A function which clamps a given value between to other values.
 */
float Clamp(float min, float max, float x){
	float y;
	if(x < min){
		y = min;
	} else if(x > max){
		y = max;
	} else{
		y = x;
	}
	return y;
}

/** Author: Zaeem Mohamed
 *  A function that calculates one iteration of a PID Control loop
 *  given a PID struct and the observed sensor value of the
 *  actuator that will effectively be controlled. The PID function is:
 *
 *  u(t) = kP*e(t) + kI*integral(e(t)dt) + kD*(de(t)/dt)
 */
float ClosedPIDStep(zmPID& pid, float obs){
	float e = pid.target - obs;
	pid.i += e * pid.dt;
	pid.d = (e - pid.e) / pid.dt;
	float u = pid.kp * e + pid.ki * pid.i + pid.kd * pid.d;

	pid.e = e;
	return u / 100.0;
}

/** Author: Zaeem Mohamed
 *  This function houses the PID Control loop that is in charge of
 *  strumming the guitar. It creates a PID object from the parameters
 *  and provides the object to the ClosedPIDStep function.
 */
void PIDStrum(float target, float kp, float ki,
			  float kd, int speed){
	float dt = 1.0 / PID_HZ;
	float targetMSPerFrame = 1000.0 * dt;
	float tp = (target<nMotorEncoder[STRUM_MOTOR]?-100:100);
	float vel = speed / 100.0;
	zmPID pid;

	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.target = target;
	pid.dt = dt;
	pid.e = pid.target - nMotorEncoder[STRUM_MOTOR];
	pid.i = 0;
	pid.d = pid.e / pid.dt;
	bool zeroed = false;
	time1[T3] = 0;
	float lastTime = time1[T3];
	while(fabs(pid.e) > E_THRES || fabs(pid.d) > D_THRES
		  && !zeroed){
		if(SensorValue(S2)){
			zeroed = true;
			nMotorEncoder[STRUM_MOTOR] = 0;
		}
		float u = ClosedPIDStep(pid, nMotorEncoder[STRUM_MOTOR]);
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

/** Author: Zaeem Mohamed
 *  This function is the same as above, except that its goal is to
 *  move the strumming more the same range as above, but instead of
 *  actually strumming, this function makes the pulling motor rotate
 *  so that the pick misses the strings. However this motor has a
 *  "less than functional" encoder so PID Control could not directly
 *  be applied to the motor. To solve this, the position of the
 *  strumming motor relative to its target set-point was used to
 *  indirectly determine where the pulling motor must be. This
 *  process is explained below.
 */
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
	float dx = target - nMotorEncoder[STRUM_MOTOR];
	zmPID pid;

	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.target = target;
	pid.dt = dt;
	pid.e = dx;
	pid.i = 0;
	pid.d = pid.e / pid.dt;
	bool zeroed = false;
	time1[T3] = 0;
	float lastTime = time1[T3];
	while(fabs(pid.e) > E_THRES || fabs(pid.d) > D_THRES
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
		float y2 = 100 * cos(PI * alpha);
		motor[PULL_MOTOR] = sign * DY * ((int)(y2) - 30);
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

/** Author: Zaeem Mohamed
 *  This function is similar to PIDStrum, except that it controls
 *  the chord switching of the robot. Used for turning the chord
 *  motor in intervals of 90 and 180 degree turns.
 */
void PIDChord(float target, float kp, float ki, float kd){
	nMotorEncoder[CHORD_MOTOR] = 0;
	float dt = 1.0 / PID_HZ;
	float targetMSPerFrame = 1000.0 * dt;
	float dx = target - 0;
	float tp = (dx < 0 ? -100 : 100);
	zmPID pid;

	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.target = target;
	pid.dt = dt;
	pid.e = dx;
	pid.i = 0;
	pid.d = pid.e / pid.dt;

	time1[T3] = 0;
	float lastTime = time1[T3];
	while(fabs(pid.e) > E_THRES || fabs(pid.d) > D_THRES){
		float u = ClosedPIDStep(pid, nMotorEncoder[CHORD_MOTOR] % 360);
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

/** Author: Zaeem Mohamed
 *  A struct that stores the data and meta data for a song.
 */
typedef struct{
	int size; // size of the song in bytes
	int tempo; // tempo of the song

	// Data of the song. Each char is 8 bits, where the first 2 bits
	// store which chord is needed to played and the remaining 6 bits
	// store the velocity of the strum.
	char frames[MAX_SONG_SIZE];
} SongData;

/** Author: Taylor Robertson
 *  This function calibrates all the motors to their fixed starting
 *  position except the chord motor.
 */
void Callibrate(){
	while(!SensorValue(S2)) motor[STRUM_MOTOR] = -15;
	motor[STRUM_MOTOR] = 0;
	motor[PULL_MOTOR] = -15;
	wait1Msec(200);
	motor[PULL_MOTOR] = 0;
	nMotorEncoder[PULL_MOTOR] = 0;
	nMotorEncoder[STRUM_MOTOR] = 0;
	nMotorEncoder[CHORD_MOTOR] = 0;
}

/** Author: Ben Krowchuk
 *  This function takes a char (from the song data) and converts it into
 *  two integers representing the chord data and strum data by applying
 *  some bit shifting and bit masking operations.
 */
void GetFrameData(char byte, int& chord, int& strum){
	chord = (int)((byte >> 6) & 0x03);
	strum = 2 * (int)((byte & 0x3F));
}

/** Author: Andrew Xu
 *  This function takes a file handle of the song info and a reference
 *  to a SongData object. It then parses that file into the SongData
 *  object by doing more bit shifts and bit masks.
 */
void GetSongData(TFileHandle fin, SongData &data){
	data.size = 0;
	int tempo;
	readIntPC(fin, tempo);
	data.tempo = tempo;
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

/** Author: Andrew Xu
 *  This function reads the chord data and previous chord data and
 *  determines what angle to turn. Calls the PIDChord function to 
 *  achieve this angle.
 */
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

/** Author: Ben Krowchuk
 *  This function receives a speed of strumming. If this speed is less
 *  than or equal to 10, the function does nothing. If it is greater, 
 *  the robot strums the guitar down twice, then up, then down, then up
 *  once more.
 */
void Strum(int speed){
	if(speed > 10){
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed * 0.25, DY);
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrum(0, KPs, KIs, KDs, speed);
		PIDStrum(STRUM_RANGE, KPs, KIs, KDs, speed);
		PIDStrumMiss(0, KPs, KIs, KDs, speed * 0.25, DY);
	}
}

//NOTE: This is global because there
//      is not enough local stack
//      space to house a buffer this
//      large.
static SongData songData; 

/** Author: Zaeem Mohamed
 *  The main function. This function first calls Calibrate.
 *  It opens the file "4chords.txt" and then passes the handle to
 *  GetSongData. It determines the target milliseconds per iteration
 *  from the song's tempo. Then the main loop is started. If the 
 *  left button is pressed, the previous chord is selected and the robot
 *  strums. If the right button is pressed, the next chord is selected
 *  and the robot strums. If the center button is selected, the song
 *  begins playing. This past of the loop uses ProcessTime to regulate 
 *  the iteration frequency. GetFrameData is called to get the chord and
 *  strum data that the robot will adhere to in that iteration. The 
 *  robot then uses that data to switch chords and strums accordingly.
 *  This is repeated until the song is done. At which point the program
 *  flow returns to the top of the main loop. The program terminates
 *  when the back button is pressed (default).
 */
task main(){
	SensorType[S2] = sensorTouch;
	bMotorReflected[motorB] = true;
	bMotorReflected[motorC] = true;
	Callibrate();
	wait1Msec(1000);
	TFileHandle fin;
	if(openReadPC(fin, "4chords.txt")){
		nxtDisplayString(5, "We Good");
		int strum, chord, lastChord = 0;
		GetSongData(fin, songData);
		float targetMSPerFrame = 60000.0 / (songData.tempo);
		for(;;){
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
