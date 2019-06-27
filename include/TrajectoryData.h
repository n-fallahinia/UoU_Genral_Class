#ifndef __TRAJECTORYDATA_H_
#define __TRAJECTORYDATA_H_

#define TRAINING_FREQUENCY 10.0

class TrajectoryData
{
public:
	float *traj[6];
	float desired_force[6];
	int current_count;
	int max_count;
	
	float old_force[6];
	float new_force[6];
	
	int ramp_count;
	int ramp_max;
	int hold_count;
	int hold_min;
	
	// Determines the trajectory type using a bitwise notation
	//      Each bit corresponds to a direction, in the order:
	//              [x y z rot-x rot-y rot-z]
	//      100000 corresponds to x-direction force control with position
	//              control in all other directions
	//      111000 corresponds to translational force control with
	//              rotational position control
	int trajectory_type;
	
	// Flag indicating whether the trajectory was read correctly
	bool flag_trajectory_ready;
	
	// Flag indicating whether the trajectory is currently running
	bool flag_started;
	
	// Flag indicating whether the trajectory has reached the "hold" phase
	bool flag_holding;
	
	// Constructor/destructor
	TrajectoryData();
	~TrajectoryData();
	
	// Read the trajectory from a data file
	void ReadTrajectoryFile(const char* filename);
	
	// Update the desired force
	//void UpdateDesiredForce(MaglevControl maglev_m);
private:

	
};

#endif
