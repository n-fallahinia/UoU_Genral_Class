#include <stdlib.h>
#include <stdio.h>
#include "TrajectoryData.h"

// Define quick-reference trajectory types
#define FORCES_ROTATIONS 56     // Default (111000)
#define POSITIONS_ROTATIONS 0   // All position control
#define FORCE_X 32              // x-direction force only
#define FORCE_Y 16              // y-direction force only
#define FORCE_Z 8               // z-direction force only

TrajectoryData::TrajectoryData()
{
	current_count = 0;
	max_count = 0;
	
	// Set flags
	flag_trajectory_ready = false;
	flag_started = false;
	flag_holding = false;
	
	// Zero out the desired forces
	for (int i=0; i<6; i++)
	{
	        old_force[i] = 0.0;
	        new_force[i] = 0.0;
	}
	
	// Set defaults for the ramp-and-hold values
	ramp_count = 0;
	ramp_max = 6;//*0.1 seconds
	hold_count = 0;
	hold_min = 2;//*0.1 seconds
        
        // Does nothing yet
        trajectory_type = FORCES_ROTATIONS;
}

TrajectoryData::~TrajectoryData()
{
	if (traj[0] != NULL)
	{
		for (int i=0; i<6; i++)
		{
			delete []traj[i];
		}
	}
}

//Load the training trajectory from the file
void TrajectoryData::ReadTrajectoryFile(const char* filename)
{
        //File open
        FILE *traj_file;
        traj_file = fopen( filename, "r" );
        
        if ( traj_file != NULL )
        {
	        //Read the number of trajectory points
	        int temp;
	        fscanf ( traj_file, "%d", &temp);
	        max_count = temp;
	        
	        for ( int i=0; i<6; i++)
	        {
		        traj[i] = new float[max_count];
	        }
	        
	        //Read in trajectory data
	        float temp_f;
	        
	        for ( int i=0; i<temp; i++)
	        {
		        for ( int j=0; j<6;j++)
		        {
			        fscanf ( traj_file, "%f", &temp_f );
			        traj[j][i] = temp_f;
		        }
	        }
	        fclose(traj_file);
        }
        else
        {
                printf("\tUnable to read trajectory file! (%s)\n", filename);
        }
        
        //Set flags
        current_count = 0;
        flag_trajectory_ready = true;
        printf("Total number of trajectory landmarks is %d\n", max_count);
} // ReadTrajectoryFile

// Update the desired trajectory
/*void TrajectoryData::UpdateTrainingTrajectory(MaglevControl maglev_m)
{
        // Baggza - We might be able to move this into the trainingRecord timer to improve speed
        // I would also like to be able to add position trajectories to this eventually
        if(flag_start_training && !flag_force_ready)
        {
                // If the force sensor is on and a training trajectory is active, update
                //   the desired force using the "ramp-and-hold"
                static float new_force[6];
                static float old_force[6];
                
                // Determine where the timing is in the "ramp-and-hold" trajectory
	        if ( current_training.ramp_count == 0 )
	        {
	                // Starting at the beginning of the ramp
	                
	                // Process each force direction to determine the new and old forces
		        printf("Moving to new force level: (");
		        for (int i=0; i<6; i++)
		        {
		                if (maglev_m.force_control_directions[i])
		                {
		                        // Read the new force from the trajectory array
			                new_force[i] = current_training.traj[i][current_training.current_count];
			                printf("%5.2f", new_force[i]);
			                
			                // If at the beginning of the trajectory, the old force is the current force.
			                //   Otherwise, it is the previous force in the trajectory array.
			                if ( current_training.current_count == 0 )
			                {
				                old_force[i] = force_sensor_m.curr_force.force[i];
			                }
			                else
			                {
				                old_force[i] = current_training.traj[i][current_training.current_count-1];
			                }
		                }
		                else
		                {
		                        // Read the new force from the trajectory array
			                new_force[i] = current_training.traj[i][current_training.current_count];
			                printf("%6.3f", new_force[i]);
			                
			                // If at the beginning of the trajectory, the old force is the current force.
			                //   Otherwise, it is the previous force in the trajectory array.
			                if ( current_training.current_count == 0 )
			                {
				                old_force[i] = maglev_m.current_position[i];
			                }
			                else
			                {
				                old_force[i] = current_training.traj[i][current_training.current_count-1];
			                }
		                }
		        }
		        printf(")\\n");
	        }
	        if (current_training.ramp_count < current_training.ramp_max && flag_image_ready == 1)
	        {
	                // The imager is ready, so the force can changes
		        for (int i=0; i<6; i++)
		        {
			        float interpolation_constant = ((float) current_training.ramp_count) / ((float) current_training.ramp_max);
			        if (maglev_m.force_control_directions[i])
			        {
		                        maglev_m.desired_force[i] = (new_force[i] - old_force[i]) * interpolation_constant + old_force[i];
	                        }
	                        else
	                        {
	                                maglev_m.desired_position[i] = (new_force[i] - old_force[i]) * interpolation_constant + old_force[i];
	                        }
		        }
		        current_training.ramp_count += 1;
	        }
	        else if (current_training.ramp_count < current_training.ramp_max + current_training.hold_min  && flag_image_ready == 1)
	        {
		        for (int i=0; i<6; i++)
		        {
		                if (maglev_m.force_control_directions[i])
		                {
			                maglev_m.desired_force[i] = new_force[i];
		                }
		                else
		                {
		                        maglev_m.desired_position[i] = new_force[i];
		                }
		        }
		        current_training.ramp_count+=1;
	        }
	        else if ( current_training.ramp_count == ( current_training.ramp_max + current_training.hold_min ) )
	        {
		        //The force is ready
		        flag_force_ready = 1;
		        current_training.ramp_count = 0;
	        }
	        //std::cout << maglev_m.desired_force[0] << " " << maglev_m.desired_force[1] << " "<< maglev_m.desired_force[2] <<"\\n";
	
        }
} // UpdateTrainingTrajectory */

