// Constants file

#ifndef UTAH_CONSTANTS
#define UTAH_CONSTANTS

// Error definitions
enum Utah_Errors
{
    UTAH_NO_ERROR = 0,
    UTAH_BAD_FILE_NAME
};

////////////////////////////////////////////////////////////////////////////////
// Mathematical constants
#define PI 3.141592

////////////////////////////////////////////////////////////////////////////////
// Maglev constants

// Number of directions used to represent pose in the MLHD
#define DATA_SIZE 6

// Number of control steps used to transition between controllers
#define NUM_TRANSITIONS 50.0

// Cutoff frequency (in Hertz) for the velocity filter
#define CUTOFF_FREQUENCY 100.0

// Frequency (in Hertz) at which to draw the Position
#define POSITION_FREQUENCY 10.0

////////////////////////////////////////////////////////////////////////////////
// BarPlot values

// Define vertical location for x-axis
#define X_AXIS 0.0

// Define title y-coordinate
#define TITLE_POSITION 9.0

// Define horizontal locations for (L)eft and (R)ight axis labels
#define L_AXIS -9.5
#define R_AXIS 7.0

// Define scales for (L)eft and (R)ight axes
#define L_SCALE 1.0
#define R_SCALE 1.0/30.0

////////////////////////////////////////////////////////////////////////////////
// TimePlot values

// Number of data points
#define NUM_DATA 200000

////////////////////////////////////////////////////////////////////////////////
// WorkspacePlot values

// Distance above and below z=0 to change colors
#define AXIS_TOL 4.0

// Maximum radius in (x,y,z) -- should replace this with the values from the maglev boundary
#define MAX_RADIUS 12.0

// Number of lines for drawing the Maglev position circle
#define NUM_LINES 36

////////////////////////////////////////////////////////////////////////////////
// main values

// Maximum value for position integral (to prevent integrator wind-up)
#define MAX_POSITION_INTEGRAL 1.0

// Maximum value for force integral (to prevent integrator wind-up)
#define MAX_FORCE_INTEGRAL 10.0

// Number of columns of data to save
#define NUM_COLUMNS 49

/////////////////////////////////////
// Trajectory data

// Stroke velocity, meters per second
#define STROKE_VELOCITY 0.005

// Stroke length, meters
#define STROKE_LENGTH 0.020

// Stroke Mean, meters
#define STROKE_MEAN 0.0

// Number of strokes to perform
#define NUM_STROKES 10

// Amount of time (in seconds) to give at the beginning of the trajectory
#define INITIAL_TIME 5.0

////////////////////////////////////////////////////////////////////////////////
// ForceSensor values

// Frequency at which to display the force readings
#define FORCE_FREQUENCY 10.0

// Not sure what these were for...
//#define RAW_TOL 9.0
//#define FORCE_TOL 0.75



#endif

