#include "main.h"

#include <iostream>
#include <math.h>

// Array to store data before saving to a file
//  NUM_DATA stored in TimePlot.h
//  NUM_COLUMNS stored in main.h
double data_array[NUM_DATA][NUM_COLUMNS];

// Hybrid controller (position & force) for the MLHD flotor
//  This will eventually control force in the z-direction and position in the
//  other 5 directions.
//  Runs a single iteration of the control loop.
void MaglevControl::PositionController(double current_time)
{
	// Declare several variables
	static double velocity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static double position_old[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static double integral[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double pid_force[6];
	static double old_time;
	double time_step, K_P, K_V, K_I, K_F, proportional;
	
	// Calculate the time step
	if (controller_counter == 0)
	{
	    // Assume zero time step
	    //  The only effect of this should be that the initial integral term
	    //  will be zero in all directions.
	    time_step = 0.0;
	}
	else
	{
	    // Calculate the new time step
	    time_step = current_time - old_time;
	}
    old_time = current_time;
	
	// Update the position
	update_position();
	
	// Process each direction
	for (int dirIdx = 0; dirIdx < 6; dirIdx++)
	{
	    // Estimate the velocity in this direction
        if (controller_counter == 0)
        {
            // Assume zero velocity since we have no prior data
            velocity[dirIdx] = 0.0;
        }
        else
        {
            // Two steps in one line:
            //      (1) Calculate velocity using first-order backwards difference derivative (portion of formula multiplied by alpha)
            //      (2) Filter the velocity using a first-order low-pass filter
            velocity[dirIdx] = alpha * ((current_position[dirIdx] - position_old[dirIdx]) / time_step) + (1.0 - alpha) * velocity[dirIdx];
        }
        
        // Extract position gains from matrix
        K_P = current_gains_position[dirIdx][0];        
        K_I = current_gains_position[dirIdx][1];
        K_V = current_gains_position[dirIdx][2];
        K_F = current_gains_position[dirIdx][3];
        
        // Calculate proportional error
        proportional = desired_position[dirIdx] - current_position[dirIdx];
        
        // Calculate integral term
        if (K_I == 0.0)
        {
            integral[dirIdx] = 0.0;
        }
        else
        {
            if (fabs(integral[dirIdx]) <= MAX_POSITION_INTEGRAL)
            {
                integral[dirIdx] += proportional * time_step;
            }
        }
        
        // Calculate controller term
        //  Feedforward term is our gravity compensation
        pid_force[dirIdx] = K_P * proportional + K_I * integral[dirIdx] - K_V * velocity[dirIdx] + K_F;
        
	    // Store the old position
        position_old[dirIdx] = current_position[dirIdx];
	} // dirIdx
	
	// Set forces
	set_forces(pid_force);
	
	// Reset if we've reached the end of the array
	if (controller_counter == NUM_DATA)
	{
	    controller_counter = 0;
	}
	
	// Store data
	data_array[controller_counter][0] = current_time;
	for (int dirIdx = 0; dirIdx < 6; dirIdx++)
	{
	    // Store desired positions in 1-6
	    data_array[controller_counter][dirIdx + 1] = desired_position[dirIdx];
	    
	    // Store actual positions in 7-12
	    data_array[controller_counter][dirIdx + 7] = current_position[dirIdx];
	    
	    // Store wrench inputs in 13-18
	    data_array[controller_counter][dirIdx + 13] = pid_force[dirIdx];
	    
	    // Store velocity values in 19-24
	    data_array[controller_counter][dirIdx + 19] = velocity[dirIdx];
	    
	    // Store integral values in 25-30
	    data_array[controller_counter][dirIdx + 25] = integral[dirIdx];
    } // dirIdx */
    
    // Increment the controller counter
	controller_counter++;
}

// Hybrid controller (position & force) for the MLHD flotor
//  This will eventually control force in the z-direction and position in the
//  other 5 directions.
//  Runs a single iteration of the control loop.
void MaglevControl::HybridController(double current_time, Reading current_force)
{
	// Declare several variables
	static double velocity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static double position_old[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	static double integral[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double pid_force[6];
	static double old_time;
	double time_step, K_P, K_V, K_I, K_F, proportional;
	
	// Calculate the time step
	if (controller_counter == 0)
	{
	    // Assume zero time step -- should have no effect
	    time_step = 0.0;
	}
	else
	{
	    // Calculate the new time step
	    time_step = current_time - old_time;
	}
    old_time = current_time;
	
	// Update the position
	update_position();
	
	// Process each direction
	for (int dirIdx = 0; dirIdx < 6; dirIdx++)
	{
	    // Estimate the velocity in this direction
        if (controller_counter == 0)
        {
            // Assume zero velocity since we have no prior data
            velocity[dirIdx] = 0.0;
        }
        else
        {
            // Two steps in one line:
            //      (1) Calculate velocity using first-order backwards difference derivative (portion of formula multiplied by alpha)
            //      (2) Filter the velocity using a first-order low-pass filter
            velocity[dirIdx] = alpha * ((current_position[dirIdx] - position_old[dirIdx]) / time_step) + (1.0 - alpha) * velocity[dirIdx];
        }
        
        // Switch for hybrid control
        if (force_control_directions[dirIdx])
        {
            ////////////////////////////////////
            // Force Controller
            
            // Extract position gains from matrix
            K_P = current_gains_force[dirIdx][0];        
            K_I = current_gains_force[dirIdx][1];
            K_V = current_gains_force[dirIdx][2];
            K_F = current_gains_force[dirIdx][3];
            
            // Calculate proportional error
            proportional = desired_force[dirIdx] - current_force.force[dirIdx];
            
            // Calculate integral term
            if (K_I == 0.0)
            {
                integral[dirIdx] = 0.0;
            }
            else
            {
                if (fabs(integral[dirIdx]) <= MAX_FORCE_INTEGRAL)
                {
                    integral[dirIdx] += proportional * time_step;
                }
            }
            
            // Calculate controller term
            //  Proportional, integral and desired terms are negative due to our sign convention
            //  Feedforward term is our gravity compensation
            pid_force[dirIdx] = -(K_P * proportional + K_I * integral[dirIdx] + desired_force[dirIdx]) - K_V * velocity[dirIdx] + K_F;
        }
        else
        {
            ////////////////////////////////////
            // Position Controller
            
            // Extract position gains from matrix
            K_P = current_gains_position[dirIdx][0];        
            K_I = current_gains_position[dirIdx][1];
            K_V = current_gains_position[dirIdx][2];
            K_F = current_gains_position[dirIdx][3];
            
            // Calculate proportional error
            proportional = desired_position[dirIdx] - current_position[dirIdx];
            
            // Calculate integral term
            if (K_I == 0.0)
            {
                integral[dirIdx] = 0.0;
            }
            else
            {
                if (fabs(integral[dirIdx]) <= MAX_POSITION_INTEGRAL)
                {
                    integral[dirIdx] += proportional * time_step;
                }
            }
            
            // Calculate controller term
            //  Feedforward term is our gravity compensation
            pid_force[dirIdx] = K_P * proportional + K_I * integral[dirIdx] - K_V * velocity[dirIdx] + K_F;
        }
        
	    // Store the old position
        position_old[dirIdx] = current_position[dirIdx];
	} // dirIdx
	
	// Set forces
	set_forces(pid_force);
	
	// Reset if we've reached the end of the array
	if (controller_counter == NUM_DATA)
	{
	    controller_counter = 0;
	}
	
	// Store data
	data_array[controller_counter][0] = current_time;
	for (int dirIdx = 0; dirIdx < 6; dirIdx++)
	{
	    // Store desired positions in 1-6
	    data_array[controller_counter][dirIdx + 1] = desired_position[dirIdx];
	    
	    // Store actual positions in 7-12
	    data_array[controller_counter][dirIdx + 7] = current_position[dirIdx];
	    
	    // Store wrench inputs in 13-18
	    data_array[controller_counter][dirIdx + 13] = pid_force[dirIdx];
	    
	    // Store velocity values in 19-24
	    data_array[controller_counter][dirIdx + 19] = velocity[dirIdx];
	    
	    // Store integral values in 25-30
	    data_array[controller_counter][dirIdx + 25] = integral[dirIdx];
	    
	    // Store desired positions in 31-36
	    data_array[controller_counter][dirIdx + 31] = desired_force[dirIdx];
	    
	    // Store actual positions in 37-42
	    data_array[controller_counter][dirIdx + 37] = current_force.force[dirIdx];
	    
	    // Store actual positions in 43-48
	    data_array[controller_counter][dirIdx + 43] = current_force.raw_force_signal[dirIdx];
    } // dirIdx */
	
    // Increment the controller counter
	controller_counter++;
} // HybridController

// Digital position controller
void MaglevControl::DigitalController(double current_time)
{
	// Declare several variables
	double pid_force[6];
	static double old_errors[6][2] = {{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0},{0.0,0.0}};
	static double old_position[6][2];
	static double old_time;
	double time_step, K_P, K_I, K_V, K_F, proportional;
	double a, b, c, d, e, f;
	
	// Update the position
	update_position();
	
	// Calculate the time step
	if (controller_counter == 0)
	{
	    // Assume zero time step
	    //  The only effect of this should be that the initial integral term
	    //  will be zero in all directions.
	    time_step = 0.0;
	    
	    // Store the old desired position on the first iteration
	    for (int dirIdx = 0; dirIdx < 6; dirIdx++)
	    {
	        old_position[dirIdx][0] = current_position[dirIdx];
	        old_position[dirIdx][1] = current_position[dirIdx];
	    } // dirIdx
	}
	else
	{
	    // Calculate the new time step
	    time_step = current_time - old_time;
	}
    old_time = current_time;
	
	// Process each direction
	for (int dirIdx = 0; dirIdx < 6; dirIdx++)
	{
	    // Extract position gains from matrix
        K_P = current_gains_position[dirIdx][0];        
        K_I = current_gains_position[dirIdx][1];
        K_V = current_gains_position[dirIdx][2];
        K_F = current_gains_position[dirIdx][3];
        
        // Calculate the digital gains
        a = 4.0*K_V + 2.0*time_step*K_P + time_step*time_step*K_I;
        b = 2.0*time_step*time_step*K_I - 8.0*K_V;
        c = 4.0*K_V - 2.0*time_step*K_P + time_step*time_step*K_I;
        d = 4.0*tau + 2.0*time_step;
        e = -8.0*tau;
        f = 4.0*tau - 2.0*time_step;
        
        // Calculate proportional error
        proportional = desired_position[dirIdx] - current_position[dirIdx];
        
        // Calculate controller term
        //  Feedforward term is our gravity compensation
        pid_force[dirIdx] = (a*proportional + b*old_errors[dirIdx][0] + c*old_errors[dirIdx][1] - e*old_position[dirIdx][0] - f*old_position[dirIdx][1]) / d + K_F;
        
	    // Update the old values
        old_position[dirIdx][1] = old_position[dirIdx][0];
        old_position[dirIdx][0] = current_position[dirIdx];
        old_errors[dirIdx][1] = old_errors[dirIdx][0];
        old_errors[dirIdx][0] = proportional;
	} // dirIdx
	
	// Set forces
	set_forces(pid_force);
	
	// Reset if we've reached the end of the array
	if (controller_counter == NUM_DATA)
	{
	    controller_counter = 0;
	}
	
	// Store data
	data_array[controller_counter][0] = current_time;
	for (int dirIdx = 0; dirIdx < 6; dirIdx++)
	{
	    // Store desired positions in 1-6
	    data_array[controller_counter][dirIdx + 1] = desired_position[dirIdx];
	    
	    // Store actual positions in 7-12
	    data_array[controller_counter][dirIdx + 7] = current_position[dirIdx];
	    
	    // Store wrench inputs in 13-18
	    data_array[controller_counter][dirIdx + 13] = pid_force[dirIdx];
	    
	    // Store old error values in 19-24
	    data_array[controller_counter][dirIdx + 19] = old_errors[dirIdx][0];
	    
	    // Store old position values in 25-30
	    data_array[controller_counter][dirIdx + 25] = old_position[dirIdx][0];
    } // dirIdx */
    
    // Increment the controller counter
	controller_counter++;
} // DigitalController

// Saving controller data to a file
void MaglevControl::save_force_data(char* force_save_file_name)
{
    // Open the file
    std::cout << "Saving the force controller data into a file\n";
    FILE *save_force_file = fopen(force_save_file_name, "w");
    std::cout << "Saving " << controller_counter << " of a possible " << NUM_DATA << " points of force data\n";
    
    // Round down to the size of the array -- should never get here
    if (controller_counter > NUM_DATA)
    {
        controller_counter = NUM_DATA;
    }
    
    if (controller_counter > 0)
    {
        // Iterate to save all recorded data
        for (unsigned int rowIdx=0; rowIdx < controller_counter; rowIdx++)
        {
            fprintf(save_force_file, "%4d %10.6f ", rowIdx, data_array[rowIdx][0]);
            for (int columnIdx = 1; columnIdx<NUM_COLUMNS; columnIdx++)
            {
                    fprintf(save_force_file, "%10.6f ", data_array[rowIdx][columnIdx]);
            } // columnIdx
            fprintf(save_force_file, "\n");
        } // rowIdx
    }
    
    // Close the file and return
    fclose(save_force_file);
    std::cout << "\tDone\n";
} // save_force_data

// Trajectory generation function
double get_desired_position(double current_time)
{
    // Calculate timing quantities
    double stroke_time = (STROKE_LENGTH / STROKE_VELOCITY);
    
    // Remove initial time
    current_time = current_time - INITIAL_TIME;
    
    // Split based on time
    if (current_time < 0.0)
    {
        // Before the beginning of the trajectory
        return STROKE_MEAN;
    }
    else if (current_time > (stroke_time * NUM_STROKES))
    {
        // Beyond the end of the trajectory
        return STROKE_MEAN;
    }
    else if (current_time < (stroke_time*0.5))
    {
        // First half-stroke up
        return STROKE_MEAN + (STROKE_VELOCITY * current_time);
    }
    else if (current_time > (stroke_time*(NUM_STROKES-0.5)))
    {
        // Final half-stroke up
        return STROKE_MEAN + (STROKE_VELOCITY * (current_time - stroke_time*(NUM_STROKES-0.5)) - (STROKE_LENGTH / 2.0));
    }
    //else
    //{
    //    // Handle all full strokes
    //    for (strokeIdx = 0; strokeIdx < 
    //}
    else if ((current_time - stroke_time*0.5) < stroke_time)
    {
        // First full down-stroke
        return STROKE_MEAN + ((STROKE_LENGTH / 2.0) - STROKE_VELOCITY * (current_time - stroke_time*0.5));
    }
    else if ((current_time - stroke_time*1.5) < stroke_time)
    {
        // First full up-stroke
        return STROKE_MEAN + (STROKE_VELOCITY * (current_time - stroke_time*1.5) - (STROKE_LENGTH / 2.0));
    }
    else if ((current_time - stroke_time*2.5) < stroke_time)
    {
        // Second full down-stroke
        return STROKE_MEAN + ((STROKE_LENGTH / 2.0) - STROKE_VELOCITY * (current_time - stroke_time*2.5));
    }
    else if ((current_time - stroke_time*3.5) < stroke_time)
    {
        // Second full up-stroke
        return STROKE_MEAN + (STROKE_VELOCITY * (current_time - stroke_time*3.5) - (STROKE_LENGTH / 2.0));
    }
    else if ((current_time - stroke_time*4.5) < stroke_time)
    {
        // Second full down-stroke
        return STROKE_MEAN + ((STROKE_LENGTH / 2.0) - STROKE_VELOCITY * (current_time - stroke_time*4.5));
    }
    else if ((current_time - stroke_time*5.5) < stroke_time)
    {
        // Second full up-stroke
        return STROKE_MEAN + (STROKE_VELOCITY * (current_time - stroke_time*5.5) - (STROKE_LENGTH / 2.0));
    }
    else if ((current_time - stroke_time*6.5) < stroke_time)
    {
        // Second full down-stroke
        return STROKE_MEAN + ((STROKE_LENGTH / 2.0) - STROKE_VELOCITY * (current_time - stroke_time*6.5));
    }
    else if ((current_time - stroke_time*7.5) < stroke_time)
    {
        // Second full up-stroke
        return STROKE_MEAN + (STROKE_VELOCITY * (current_time - stroke_time*7.5) - (STROKE_LENGTH / 2.0));
    }
    else if ((current_time - stroke_time*8.5) < stroke_time)
    {
        // Second full down-stroke
        return STROKE_MEAN + ((STROKE_LENGTH / 2.0) - STROKE_VELOCITY * (current_time - stroke_time*8.5));
    }
    else if ((current_time - stroke_time*9.5) < stroke_time)
    {
        // Second full up-stroke
        return STROKE_MEAN + (STROKE_VELOCITY * (current_time - stroke_time*9.5) - (STROKE_LENGTH / 2.0));
    }
    else
    {
        // Eventually, this will be populated so the function will never get here
        return STROKE_MEAN + (-5.0);
    }
} // get_desired_position

