// A SHOT on Goal. General Version. DO NOT USE FOR ASSIGNMENT.

/* In the game of rugby a touchdown is converted by booting the ball over the 3m high goal cross bar from some point
on the pitch, but orthogonal to the try line where the touchdown occured.
This program calculates how fast and at what vertical angle you would have to kick the ball to get it over the cross bar from
a set distance (which is input).
It does this by cycling through a series of angles, and figuring out using standard trajectory equations
whether the ball would make it over the bar (ignoring factors such as air resistance, ball spin, air
density, and so on) until the first working combination is found. Using the kick speed and angle thus found, a series of x:y
coordinates of the ball's trajectory path are calculated and saved in an array. It is assumed that the ball is kicked on target
and goes through the uprights.
A 'display' function then plots these on a rugby-stylised graph for inspection (see screen layout later).
Units are (m) metres, (s) seconds, (deg) degrees, all float values.

Adrian Oram, March 2017 (not a rugby supporter, so don't ask!)
*/

#define yourName     "Adrian Oram"				// Please change these as appropriate!
#define yourTeamName "Fulchester RC"

#define _PS3		// Build for PS3 system otherwise x86 if commented out. (You will need to create an x86 project and paste all required files across.)
//#define _trace	// comment out to remove trace output used for testing
//#define _longTrace	// comment out to avoid lengthy trace of trial height calculations

#ifdef _PS3
#include <sys/sys_time.h>	// for PS3 timers  
#else
#include "hr_time.h"		//for x86 timers 
#include <fstream>			// for file I/O (tricky on PS3)
#endif // _PS3

#include <math.h>			// for COS, TAN, etc
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
using namespace std;

// Function Prototypes
template <class T>
string ToString(T);
void getDistanceToKick(float*);
bool findSHOTonGoalSpeedAndAngle(float*, float*, float);
void generateFlightPath(float, float);
void showFlightPathResults(float, float, float);

// Timing stuff
#ifdef _PS3
long long get_time_in_ticks(void);		// home brew system clock reader (1 line of assembly).
long long start, stop;					// Use this with Time Base register method of timing
const double ticksToSeconds (79.8F);	// PS3 clocks at 3.2 GHz this is 24.9x that.
#else
CStopWatch timer;				// x86
#endif // !_PS3

const int repeats = 10000;				// Repeat timings to help avoid exceptions, and record fastest times seen.
// On PS3 should be less of a problem as little else running.

long long SpeedAndAngleTime(1000000);	// Set high intially.
long long GenerateFlightPathTime(1000000);


// Some constants
const float g(9.81F);					// (m/s/s) gravity
const float Pi(3.14159265358979323846F);// This value stolen from M_PI defines in Math.h) - used to convert degrees to radians
const float dataEnd = -1.0F;			// End of data marker
const int x = 0;						// coordinate system
const int y = 1;
#define SPACE ' '
const double micro(1.0e6F);				// scaling factor to give microseconds
const double milli(1.0e3F);				// scaling factor to give milliseconds


// Kick metrics
const float minAngle(15.0F);	// (deg) pretty low!
const float maxAngle(45.0F);	// (deg) pretty steep. It becomes increasingly difficult to put enough energy into a ball at angles above 45 deg.
const float minSpeed(5.0F);		// (m/s) pretty pathetic!
const float maxSpeed(32.0F);	// (m/s) who let Superman on the pitch!? Research indicates that 26 +/-1.7 m/s is optimal kick speed.
const float maxHeight(8.5F);	// (m)   trajectories above this height can't be displayed (out the park!)
float kickSpeed;				// (m/s) calculated
float kickAngle;				// (deg) calculated

// Pitch metrics
const float minDistanceToGoal(5.0F);	// (m) Probably too close! Might get charged down!
const float maxDistanceToGoal(50.0F);	// (m) This is almost half a standard rugby pitch length.
const float crossBarHeight(3.0F);		// (m) 3m is the standard rugby cross bar height.
const float margin(0.5F);				// (m) allow for a margin of error getting it over.
const float goalPostHeight(7.0F);		// (m) Upto 16m usually. Don't make higher, it won't fit on display!
float distanceToGoal;					// (m) Kicking distance, input by user.

// Final flight path of ball as a series of x,y coordinates
const float deltaD(0.5F);			// (m) need a coordinate every tick of this distance, plus increment used for speed and angle.
const float deltaY(0.25F);			// (m) increment in the height direction (vertical exaggeration of x2)
const float yScale = 1.0F / deltaY;	// vertical scaling factor
const float xScale = 1.0F / deltaD;	// horizontal scaling factor

const int maxDataPoints = (int)((maxDistanceToGoal + 2.0F) / deltaD);	// =104, calculate a data point for each 0.5 metre along
// +2.0 so that ball appears beyond the goal at maxDistance

float flightPath[104 + 1][2];			// x,y coords (m,m) of ball flight. The sequence terminates with 'dataEnd' if fewer than maxDataPoints used.

//************************************* MAIN ***********************************************************************
int main(void)
{
	bool foundCombo(false);

	getDistanceToKick(&distanceToGoal);

	cout << "\nYou entered " << distanceToGoal << " metres. Looking for solution for kick speed and angle...";
	fflush(stdout);	//PS3 console fix

	for (int reps(0); reps < repeats; ++reps)
	{
#ifdef _PS3
		start = get_time_in_ticks();		//*** PS3
#else
		timer.startTimer();					//*** x86
#endif
		//*********************************************************************************************************
		foundCombo = findSHOTonGoalSpeedAndAngle(&kickSpeed, &kickAngle, distanceToGoal);

		//*********************************************************************************************************
#ifdef _PS3
		stop = get_time_in_ticks();				//*** PS3 Time Base register returns a tick count
		if ((stop - start) < SpeedAndAngleTime) SpeedAndAngleTime = (stop - start);				//*** PS3 record fastest time
#else
		timer.stopTimer();					//*** x86 timers return time in Seconds, so scale to microseconds
		if (timer.getElapsedTime() < SpeedAndAngleTime) SpeedAndAngleTime = (timer.getElapsedTime() * micro);	//*** x86 record fastest time
#endif
	}

	if (foundCombo)
	{
		cout << " solution found. Generating ball's flight path for display...";
#ifdef _trace
		cout << "\nSpeed=" << kickSpeed << " Angle =" << kickAngle;
#endif // _trace

		for (int reps(0); reps < repeats; ++reps)
		{
#ifdef _PS3
			start = get_time_in_ticks();		//*** PS3
#else
			timer.startTimer();					//*** x86
#endif
			//*********************************************************************************************************
			generateFlightPath(kickSpeed, kickAngle);

			//*********************************************************************************************************
#ifdef _PS3
			stop = get_time_in_ticks();			//*** PS3 Time Base register returns a tick count
			if ((stop - start) < GenerateFlightPathTime) GenerateFlightPathTime = (stop - start);				//*** PS3 record fastest time
#else
			timer.stopTimer();					//*** x86 timers return time in Seconds, so scale to microseconds
			if (timer.getElapsedTime() < GenerateFlightPathTime) GenerateFlightPathTime = (timer.getElapsedTime() * micro);	//*** x86 record fastest time
#endif
		}
		showFlightPathResults(kickSpeed, kickAngle, distanceToGoal);
	}
	else cout << "no solution found.\n";

	cout << "\nDone...";
#ifndef _PS3
	system("PAUSE");	// hold console on PC, OK to return on PS3 as console display persists.
#endif

	return(0);
}
//************************************* END MAIN ********************************************************************


// Figure out the first working combination of Speed and Angle.
// Using the lowest kick speed to begin with, increment through the angles from low to high until one is found that gets 
// the ball over the bar. If none found increase the speed and repeat. Rationale: it's better to use as little 
// energy in the kick as possible! DeltaD is used as the increment for both angle and speed.

bool findSHOTonGoalSpeedAndAngle(float* speed, float* angle, float x)
{
	// *** reminders ***
	//const float minAngle(15.0)
	//const float maxAngle(45.0)
	//const float minSpeed(5.0)
	//const float maxSpeed(32.0)
	//const float crossBarHeight(3.0)

	float nextSpeed;
	float nextAngle(minAngle);	// Start with shallowest angle...
	float height;

	bool foundCombo(false);		// Found combination of speed and angle that gets ball over bar?

	while (!foundCombo && !(nextAngle > maxAngle))				// Think de Morgan's Theory, perhaps.
	{
		float AngleRads = (nextAngle * (Pi / 180.0F));			// Need radians for cos and tan functions
		nextSpeed = minSpeed;									// reset minimum speed 

		while (!foundCombo && !(nextSpeed > maxSpeed))
		{
			// Figure out height of ball at goal post distance (x), using classic trajectory equation...
			//
			//			height =  (-g * x^2)/(2 * cos(angle)^2 * speed^2) + (x * tan(angle))
			//
			// If this is > cross bar height (plus any margin allowed) then result! 
			// Note: height could become negative if ball hits ground short of posts (and theoretically keeps going underground!).
			// Note: Max horizontal distance can be calculated from = (speed^2) * sin(2*angle)/g

			height = ((-g * x*x) / (2.0F *cos(AngleRads) *cos(AngleRads) * (nextSpeed*nextSpeed))) + (x *tan(AngleRads));	//Phew!
			
#ifdef _longTrace  // echo to screen as calculations proceed (can be lengthy, be patient!)
			cout << setw(4) << setprecision(4) << "\nHeight found for speed " << nextSpeed << "m/s\t\t= " << height << " m,\t\tkicking at angle " << nextAngle << " degrees";
#endif //_longTrace

			if (height > crossBarHeight + margin)	// Success! 
			{
				*speed = nextSpeed;			// Record the working combination...
				*angle = nextAngle;
				foundCombo = true;			// ... and stop looking.
			}
			else nextSpeed += deltaD;		// Otherwise try next speed up (+0.5 m/s).
		}
		nextAngle += deltaD;	// no joy, try next angle up (+0.5 degrees).
	}
	return (foundCombo);
}

// With metrics found, calculate the flight path coords. Uses 'flightPath[104][2]' array as global.
void generateFlightPath(float speed, float angle)
{
	//  ...reminders...
	//const float deltaD (0.5F)
	//const int maxDataPoints = 104
	//const float maxHeight(8.5F);	// (m) trajectories above this height can't be displayed (out the park!)

	float yValue(0.001F);	// ball is sitting on a tee just above the ground to begin with, of course!
	float xValue(0.0F);		// ...and hasn't moved yet.
	const float AngleRads = (angle * (Pi / 180.0F));	// Need radians for cos and tan functions 

	int i(0);
	for (; i < maxDataPoints && (yValue > 0.0) && (yValue <= maxHeight); ++i)	// If height goes negative or too high, STOP!
	{
		flightPath[i][x] = xValue;	// store data points
		flightPath[i][y] = yValue;
		xValue += deltaD;			// do for each increment tick across the pitch

		// find the 'y' (height) for each 'x' distance using the angle and speed previously found (same equation as above)
		yValue = ((-g * xValue * xValue) / (2.0F * cos(AngleRads) * cos(AngleRads) * (speed * speed))) + (xValue * tan(AngleRads));
	}
	// Finished generating required data points, now mark end-of-data with -1.0 (dataEnd)
	flightPath[i][x] = dataEnd;
	flightPath[i][y] = dataEnd;
}

//************************************ Supporting functions *******************************************************
//*****************************************************************************************************************

// Display  a stylised flightpath of ball etc. Memory is cheap so lots of replicated chars used for simplicity. 
// The string array below holds all the fixed text - the data points and other information are superimposed on it.
// The graph displays with roughly a 5x vertical exaggeration, the scale ticks have a 2x vertical exaggeration.
// PLEASE DON'T ALTER ANYTHING IN THIS FUNCTION.

void showFlightPathResults(float speed, float angle, float distanceToGoal)
{
	string YaxisTitle1 = "\n\n Height (m) above pitch\n in 0.25m increments\t\t\t\t\t\tSHOT ON GOAL CALCULATOR ";
	string YaxisTitle2 = "                           \n (Vert. exag.~= x5) \t\t\t\t\t\t~~~~~~~~~~~~~~~~~~~~~~~\n\n";
	string XaxisTitle1 = "\n\t\t\tDistance (m) to ";
	string XaxisTitle2 = "'s goal in 0.5m increments\n\n";
	string goalPostTitle = "+ = cross bar";

	const int leftMargin = 13;		// Number of chars from left side of screen to '0' position, from where ball is kicked.
	const int bottomMargin = 2;		// ditto from bottom to pitch level.
	const int resultsTextPos = 29;	// How far in to position results data.
	const int graphLines = 37;		// How many lines of text there are in the array below.
	const int pitchLevel = graphLines - bottomMargin - 1; // get stuff on to pitch directly.

	// This array = 4.4KB ish. There are 121 characters in each line + '\0' terminator.

	// This string array is treated as a 2-D array of chars, with row [0] at the top. The 'X's are replaced by
	// the performance data.

	string graphDisplay[] = {

		" 8.5 | Time to solution (us)  X                                                                                         \n",
		"     | Time generating  (us)  X                                                                                         \n",
		" 8.0 |      Kick angle (deg)  X             .         .         .         .         .         .         .         .     \n",
		"     |      Kick speed (m/s)  X                                                                                         \n",
		" 7.5 |         X  Iterations                                                                                           \n",
		"     |                                                                                                                 \n",
		" 7.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 6.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 6.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 5.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 5.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 4.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 4.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 3.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 3.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 2.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 2.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 1.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 1.0 |  _Q             .         .         .         .         .         .         .         .         .         .     \n",
		"     | | |\\_o                                                                                                          \n",
		" 0.5 | o |____#                                                                                                        \n",
		"     |  /                                                                                                              \n",
		" 0.0 | /    BLAT!!                                                                                                     \n",
		"     +-##----^+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++\n",
		"             0         5         10        15        20        25        30        35        40        45        50    \n"
	};// Nb. the double '\\' for the kicker's arm isn't a mistake. See actual output...

	const char goalPost = '|';
	const char crossBar = '+';
	const char ball = 'O';

	// Insert timing and other data...
	const int fieldWidth = 6;	//...convert data to a string and insert in display; SPACE replaces the end NULL of the string.
	graphDisplay[0].replace(resultsTextPos, fieldWidth, ToString((double)SpeedAndAngleTime/ticksToSeconds) + SPACE);		// Time for finding the solution
	graphDisplay[1].replace(resultsTextPos, fieldWidth, ToString((double)GenerateFlightPathTime/ticksToSeconds) + SPACE);	// Time for generating data points
	graphDisplay[2].replace(resultsTextPos, fieldWidth, ToString(angle) + SPACE);		// Kick angle found
	graphDisplay[3].replace(resultsTextPos, fieldWidth, ToString(speed) + SPACE);		// Kick speed found
	graphDisplay[4].replace(11, fieldWidth, ToString(repeats) + SPACE);		// # of times calcs repeated

	// Insert trajectory data from flightPath array into the array above. Scale the height values then convert to INTs for array indexing.
	for (int i(0); i < maxDataPoints && flightPath[i][x] != dataEnd; i++)
	{
		graphDisplay[pitchLevel - (int)(yScale*flightPath[i][y])]
			[leftMargin + (int)(xScale*flightPath[i][x])] = ball;  // round the data to act as indexes to string array
	};

	int GoalPostXpos = (int)(xScale * distanceToGoal) + leftMargin; // absolute X coord of goal posts.

	// Insert goal post at required distance into array above.
	for (int i(0); i < (int)(goalPostHeight * yScale); i++)
	{
		graphDisplay[pitchLevel - i][GoalPostXpos] = goalPost;	// insert from ground upwards (watch it go up in a memory window!)
	};

	// Place the cross bar...
	graphDisplay[pitchLevel + 1 - (int)(crossBarHeight * yScale)][GoalPostXpos] = crossBar;

	// Produce the finished graph on the console...
	cout << YaxisTitle1 << '(' << yourName << ')' << YaxisTitle2;
	for (int i(0); i<graphLines; ++i) cout << graphDisplay[i];	// display the big picture...

	// Output enough spaces to get goal post title to correct position, aligning the '+' with the post...
	for (int i(0); i < GoalPostXpos; i++) cout << SPACE; cout << goalPostTitle;

	// Last but not least...
	cout << XaxisTitle1 << yourTeamName << XaxisTitle2;

	//*************************************************************************************
#ifdef _trace	// Display/save generated data...

#ifndef _PS3		// can only access files easily on PC
	ofstream flightData;
	flightData.open("FlightPathData.txt", ios::app);
	flightData << "\n\nFlight path data for distance " << distanceToGoal << "m, angle= " << angle
		<< " degrees, and speed= " << speed << " m/s:\n";
	flightData << setw(3);
#endif // !_PS3
	cout << "\n\nFlight path data:\n";
	cout << setw(3);
	for (int i(0); i < maxDataPoints && flightPath[i][x] != dataEnd; ++i)
	{
		cout << '{' << flightPath[i][x] << ',' << flightPath[i][y] << "},";
#ifndef _PS3
		flightData << '{' << flightPath[i][x] << ',' << flightPath[i][y] << "},";
#endif // !_PS3
	}
	cout << "\n\nGraph points affected:\n";
#ifndef _PS3
	flightData << "\n\nGraph points affected:\n";
#endif // !_PS3
	for (int i(0); i < maxDataPoints && flightPath[i][x] != dataEnd; ++i)
	{
		cout << '{' << (leftMargin + (int)(xScale*flightPath[i][x])) << ',' << (pitchLevel - (int)(yScale*flightPath[i][y])) << "},";
#ifndef _PS3
		flightData << '{' << (leftMargin + (int)(xScale*flightPath[i][x])) << ',' << (pitchLevel - (int)(yScale*flightPath[i][y])) << "},";
#endif // !_PS3
	}
#ifndef _PS3
	flightData.close();
#endif // !_PS3
#endif //_trace
	//*************************************************************************************
}


//*****************************************************************************************************************
void getDistanceToKick(float* distance)
{
	cout << "\nDistance to goal in metres, 5.0 min, 50.0 max?:\n";
	cin >> *distance;
	while (*distance < minDistanceToGoal || *distance > maxDistanceToGoal)
	{
		cout << "\nOutside acceptable range. Try again:";  cin >> *distance;
	}
}

//*****************************************************************************************************************
template <class T>					// converts INTs and FLOATs to a string of chars
string ToString(T some_value)
{
	ostringstream textVersion;
	textVersion << some_value;
	return textVersion.str();
}
//*****************************************************************************************************************
#ifdef _PS3
long long get_time_in_ticks()		// Read the PS3's time base (TB) register and return the low 32 bits in R3
{									// Note this isn't in seconds as it depends on frequency of update.
	asm volatile (" mftb 3 \n"		// It does seem that dividing by 79.8 would give micro seconds.
					:::"r3");		// R3 is the default return parameter (uses clobber list only)
};
#endif
//************************************* END OF PROGRAM ************************************************************

