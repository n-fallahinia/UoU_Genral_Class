// Basic functions for interacting with the MLHD
// 
// There should be no need to modify this file once it is created.
// It could be replaced by a class.
// 

#include "MaglevControl.h"
#include "Constants.h"

#include <iostream>

#include "maglev_parameters.h"

/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables for use with the Maglev

// Define address of MagLev Server
const char* maglev_server_name = "192.168.0.2";

// Information for Gain Arrays:
//     Rows (index 0) correspond to directions (x, y, z, theta_x, theta_y, theta_z)
//     Columns (index 1) correspond to gains (Kp, Ki, Kd/v, Kff)
//
//     Force controllers have unity feed-forward gain, so no need to store a separate Kff

// Default directions for force control
const bool default_force_control_directions[6] = {false, false, true, false, false, false};

// PIDF gains used for the internal position controller
// These are loaded using the ml_SetGainVecAxes function
const double default_gains_internal[][4] = {
                        { 2400.0, 0.0, 15.00, 0.0},
                        { 2400.0, 0.0, 15.00, 0.0},
                        {10000.0, 0.0, 15.00, 3.5},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   10.0, 0.0,  0.15, 0.0}};

// PIVF gains used for the "new" position controller
// This is our controller
const double default_gains_position[][4] = {
                        { 2000.0,10.0, 15.00, 0.000000},
                        { 2000.0, 0.0, 15.00, 0.000000},
                        { 5000.0, 0.0, 15.00, 5.673596},
                        {   25.0, 0.0,  0.20, 0.000000},
                        {   20.0, 0.0,  0.25, 0.000000},
                        {   10.0, 0.0,  0.15, 0.000000}};

// PIV force controller gains
// This is our controller
const double default_gains_force[][4] = {
                        { 0.5,20.0, 5.0,0.00},
                        { 0.5,20.0, 5.0,0.00},
                        { 0.5,40.0,20.0,5.67},
                        { 0.0, 0.0, 0.0,0.00},
                        { 0.0, 0.0, 0.0,0.00},
                        { 0.0, 0.0, 0.0,0.00}};

// PIDF gains used for the other internal controllers (BOUNDARY, LOCKED,
//      CONSTRAINED).  These are loaded using the ml_SetGainVecAxes function.
//      The feed-forward gains are only used for the LOCKED axis.
const double default_gains_other[][4] = {
                        { 4800.0, 0.0, 15.00, 0.0},
                        { 4800.0, 0.0, 15.00, 0.0},
                        {15000.0, 0.0, 15.00, 3.5},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   50.0, 0.0,  0.15, 0.0},
                        {   15.0, 0.0,  0.15, 0.0}};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants defining the size of the flotor workspace boundary
const double position_boundary = 0.012;
const double rotation_boundary = PI * 8.0 / 180.0;
const double maglev_boundary_left[6] = {-position_boundary, -position_boundary, -position_boundary, -rotation_boundary, -rotation_boundary, -rotation_boundary};
const double maglev_boundary_right[6] = {position_boundary, position_boundary, position_boundary, rotation_boundary, rotation_boundary, rotation_boundary};
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback Handler declarations
//      Set up here as external functions since the Maglev API cannot handle
//              them as part of the class (or at least I can't figure out how to
//              make it accept them as part of the class).
extern int tick_callback_handler( ml_device_handle_t maglev_handle,  ml_position_t *maglev_position );

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for use with the Maglev

// Constructor
MaglevControl::MaglevControl()
{
    std::cout << "MaglevControl Constructor\n";
    
    // Set up device handle for communicating with the MLHD
    device_handler = new ml_device_handle_t;
    
    // Always use standard type of gains (as opposed to border, etc.)
    gainset_type = ML_GAINSET_TYPE_NORMAL;
    
    // Read gains from the data file
    gain_file_name = "force_gains.txt";
    read_file_gains();
    
    // Set desired matrices to zero
    for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
    {
        desired_position[dirIdx] = 0.0;
        desired_force[dirIdx] = 0.0;
        
        // Initialize gain matrices to zero
        for (int gainIdx = 0; gainIdx < 4; gainIdx++)
        {
            current_gains_internal[dirIdx][gainIdx] = 0.0;
            current_gains_position[dirIdx][gainIdx] = 0.0;
            current_gains_force[dirIdx][gainIdx] = 0.0;
            starting_gains_internal[dirIdx][gainIdx] = 0.0;
            starting_gains_position[dirIdx][gainIdx] = 0.0;
            starting_gains_force[dirIdx][gainIdx] = 0.0;
            desired_gains_internal[dirIdx][gainIdx] = 0.0;
            desired_gains_position[dirIdx][gainIdx] = 0.0;
            desired_gains_force[dirIdx][gainIdx] = 0.0;
        } // gainIdx
    } // dirIdx
    
    // Initialize the controller counter
    controller_counter = 0;
    start_counter = 0;
    
    // Our controller is not running at the beginning
    using_external_control = false;
    in_transition = false;
    use_trajectory = false;
} // Standard MaglevControl Constructor

// Destructor
MaglevControl::~MaglevControl()
{
    std::cout << "MaglevControl Destructor\n";
}

// Connect to the MLHD
bool MaglevControl::connect()
{
    // Connect to the MLHD
    std::cout << "Trying to connect to the Maglev";
    if (ml_Connect ( device_handler, maglev_server_name ) != ML_STATUS_OK)
    {
        std::cout << "Failed connecting to" << maglev_server_name << "Wrong server address? Server down?\n";
        return false;
    }
    std::cout << "...Success!\n";
    
    // Register the Tick Callback handler
    if (ml_RegisterCallbackTick(*device_handler, tick_callback_handler) != ML_STATUS_OK)
    {
        printf("\n****************************************\n\tFailed to properly register the Tick Callback handler!\n****************************************\n\n");
    }
    
    // Retrieve the servo frequency
    if (ml_GetServoFrequency( *device_handler, &control_freq) != ML_STATUS_OK)
    {
        std::cout << "Error while collecting the Servo Frequency!\n";
        return false;
    }
    
    // Set up the velocity filter parameters
    tau = 1.0 / (2.0 * PI * CUTOFF_FREQUENCY);
    alpha = tau / (tau + (1.0/control_freq));
    
    // Take off, if possible
    double minPos = -10.0;
    timespec half_second;
    half_second.tv_sec = 0;
    half_second.tv_nsec = 500000000L;
    while(minPos < -position_boundary)
    {
        // Define a Fault tracking variable and initialize it with a fault
        ml_fault_t current_fault;
        current_fault.value = 1;
        
        // Loop until no fault condition exists
        while(current_fault.value != 0)
        {
            // Read faults from the MLHD and reset the fault conditions
            ml_GetFault( *device_handler, &current_fault );
            ml_ResetFault(*device_handler);

            // Wait 0.5 seconds
            nanosleep(&half_second, (struct timespec *)NULL);

            // Echo a message to the user indicating that the flotor needs to be adjusted
            std::cout << "Cannot take off, sensor out of range or some other fault: " << current_fault.value << " trying again..\n";
        }
        
        // Attempt to take off
        std::cout << "Attempting to take off...\n";
        ml_Takeoff(*device_handler);
        
        // Check the position of the MLHD
        update_position();
        minPos = minValue(current_position);
        
        // If it's outside the workspace boundary, report failure to take off
        if(minPos < -position_boundary)
        {
            std::cout << "Failed" << minPos <<"\n";
        }
        else
        {
            std::cout << "Success" << minPos << "\n";
        }
    }
    
    return true;
} // connect

// Turn off the MLHD (Land and Disconnect)
void MaglevControl::turn_off ()
{
    // Unregister the Tick Callback handler
    if (ml_UnregisterCallbackTick(*device_handler) != ML_STATUS_OK)
    {
        std::cout << "\n****************************************\n\tFailed to properly unregister the Tick Callback handler!\n****************************************\n\n";
    }
    else
    {
        std::cout << "\tUnregistered the MLHD Tick Callback handler\n";
    }
    
    // Define the file in which to save the force data
    char output_file[200];
    sprintf(output_file, "data.txt");
    
    // Save the data, if it's still being recorded
    save_force_data(output_file);
    
    // Land the maglev
    if (!outside_boundary())
    {
        std::cout << "Landing the flotor...";
        ml_Land(*device_handler);
        std::cout << "Done!\n";
    }
    
      // Disconnect from server
      std::cout << "Disconnecting from the MLHD...";
      ml_Disconnect(*device_handler);
      std::cout << "Done!\n";
} // turn_off

// Get the current position reading from the maglev
void MaglevControl::update_position()
{
    ml_position_t pActual;
    ml_GetActualPosition( *device_handler, &pActual );
    for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
    {
        current_position[dirIdx] = pActual.values[dirIdx];
    }
} // update_position

// Get the current set of forces and torques of maglev
void MaglevControl::get_force()
{
        ml_GetForces(*device_handler, &maglev_forces);
} // get_force

// Get the current set of coil currents of maglev
void MaglevControl::get_current ()
{
        ml_GetCurrents(*device_handler, &maglev_currents);
} // get_current

// Get the current set of coil temperatures of maglev
void MaglevControl::get_temperature()
{
        ml_GetTemp(*device_handler, &maglev_temperatures);
} // get_temperature


// Store the position in the input vector
void MaglevControl::get_position(double * position)
{
    update_position();
    for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
    {
        position[dirIdx] = current_position[dirIdx];
    }
}

// Return the position in a given direction
double MaglevControl::get_position(int dirIdx)
{
    update_position();
    if ((dirIdx < 0) || (dirIdx >= DATA_SIZE))
    {
        return 0.0;
    }
    return current_position[dirIdx];
} // get_position

// Send force commands to the MLHD
void MaglevControl::set_forces(double * pid_force)
{
    ml_forces_t input_force;
    for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
    {
        input_force.values[dirIdx] = pid_force[dirIdx];
    }
    ml_SetForces(*device_handler, input_force);
} // set_forces

// Calculate the minimum value of an array or an ml_position_t structure
inline double MaglevControl::minValue(double my_array[DATA_SIZE])
{
    double current_minimum = my_array[0];
    for(int dirIdx=1; dirIdx < DATA_SIZE; dirIdx++)
    {
        if(my_array[dirIdx] < current_minimum)
        {
            current_minimum = my_array[dirIdx];
        }
    }
    return current_minimum;
} // minValue
inline float MaglevControl::minValue(ml_position_t my_position)
{
    float current_minimum = my_position.values[0];
    for(int dirIdx=1; dirIdx < DATA_SIZE; dirIdx++)
    {
        if(my_position.values[dirIdx] < current_minimum)
        {
            current_minimum = my_position.values[dirIdx];
        }
    }
    return current_minimum;
} // minValue

// Determine whether the Maglev flotor is currently outside the boundary
bool MaglevControl::outside_boundary()
{
    bool outside_boundary = false;
    
    // Detect whether a boundary-violation fault has occurred
    ml_GetFault(*device_handler, &current_fault);
    if(current_fault.value != ML_FAULT_TYPE_CLEAR)
    {
        // If a fault has occurred, determine whether one of the
        //      components is an out-of-range fault
        if (current_fault.value >= ML_FAULT_TYPE_FLOTOR_OVERSPEED)
        {
            // Flotor moving too fast
            current_fault.value -= ML_FAULT_TYPE_FLOTOR_OVERSPEED;
        }
        if (current_fault.value >= ML_FAULT_TYPE_COIL_OVERCURRENT)
        {
            // Coil current too high
            current_fault.value -= ML_FAULT_TYPE_COIL_OVERCURRENT;
        }
        if (current_fault.value >= ML_FAULT_TYPE_COIL_OVERTEMP)
        {
            // Coil too hot
            current_fault.value -= ML_FAULT_TYPE_COIL_OVERTEMP;
        }
        if (current_fault.value >= ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE)
        {
            // Sensor out of range - Exactly what we're looking for
            outside_boundary = true;
        }
    }
    else
    {
        // Record the Maglev's actual position
        update_position();
        
        // Loop to detect whether the actual position is outside the specified boundary
        int dirIdx = 0;
        while ((!outside_boundary) && (dirIdx < DATA_SIZE))
        {
            // Check if the position and orientation are valid
            if ((current_position[dirIdx] < maglev_boundary_left[dirIdx]) || (current_position[dirIdx] > maglev_boundary_right[dirIdx]))
            {
                outside_boundary = true;
            }
            
            // Increment the counter
            dirIdx++;
        }
    }
    
    return (outside_boundary);
} // maglevOutsideBoundary

// Sets the desired position of the MLHD
void MaglevControl::set_desired_position(double position[DATA_SIZE])
{
    for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
    {
        desired_position[dirIdx] = position[dirIdx];
    }
} // set_desired_position

// Set the desired wrench vector
void MaglevControl::set_desired_forces(double desired_wrench[DATA_SIZE])
{
    // Set each direction independently
    for (int forceIdx = 0; forceIdx < DATA_SIZE; forceIdx++)
    {
        desired_force[forceIdx] = desired_wrench[forceIdx];
    } // forceIdx
    std::cout << "***********************************************************************\n";
    std::cout << "***********************************************************************\n";
    std::cout << "***********************************************************************\n";
    std::cout << "Desired force changed! (" << desired_force[2] << ")\n";
    std::cout << "***********************************************************************\n";
    std::cout << "***********************************************************************\n";
    std::cout << "***********************************************************************\n";
} // set_desired_forces

//Set the position of maglev (uses the internal position controller)
void MaglevControl::update_internal_desired_position()
{
    // Move the desired position values into an MLHD API position structure
    ml_position_t des_pos;
    for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++ )
    {
        // Verify that the position and orientation are within the limits
        if ((desired_position[dirIdx] > maglev_boundary_left[dirIdx]) && (desired_position[dirIdx] < maglev_boundary_right[dirIdx]))
        {
            des_pos.values[dirIdx] = desired_position[dirIdx];
        }
        else
        {
            // Set the desired position to the edge of the boundary
            //std::cout << "The new position is out of boundary \n";
            if (desired_position[dirIdx] <= maglev_boundary_left[dirIdx])
            {
                des_pos.values[dirIdx] = maglev_boundary_left[dirIdx];
            }
            else if (desired_position[dirIdx] >= maglev_boundary_right[dirIdx])
            {
                des_pos.values[dirIdx] = maglev_boundary_right[dirIdx];
            }
        }
    } // dirIdx
    
    // Send the desired position values to the MLHD
    ml_SetDesiredPosition(*device_handler, des_pos);
} // update_internal_desired_position

// Retrieve the current value of the internal gain matrix at the given position
double MaglevControl::get_internal_gain(int dirIdx, int gainIdx)
{
    //return current_gains_internal[dirIdx][gainIdx];
    return current_gains_position[dirIdx][gainIdx];
} // get_internal_gain

// Retrieve the internal gains into the current_gains_internal matrix
void MaglevControl::update_internal_gains()
{
    // Retrieve the current Gain Vector
    if (ml_GetGainVecAxes (*device_handler, gainset_type, &gain_vector) != ML_STATUS_OK)
    {
        std::cout << "Fatal ERROR -- Cannot get the gains from the maglev!\n";
        return;
    }
    
    // Extract each direction's gains into our matrix
    for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
    {                
        current_gains_internal[dirIdx][0] = gain_vector.values[dirIdx].p;
        current_gains_internal[dirIdx][1] = gain_vector.values[dirIdx].i;
        current_gains_internal[dirIdx][2] = gain_vector.values[dirIdx].d;
        current_gains_internal[dirIdx][3] = gain_vector.values[dirIdx].ff;
    } // dirIdx
} // maglevUpdateInternalGains

// Set the gains for the internal position controller
void MaglevControl::set_internal_gains()
{
    // Iterate through each direction to set the gains
    for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
    {
        // Assign Proportional, Integral, Derivative and Feed-Forward
        //      gains in this direction
        gain_vector.values[dirIdx].p  = current_gains_internal[dirIdx][0];
        gain_vector.values[dirIdx].i  = current_gains_internal[dirIdx][1];
        gain_vector.values[dirIdx].d  = current_gains_internal[dirIdx][2];
        gain_vector.values[dirIdx].ff = current_gains_internal[dirIdx][3];
    } // dirIdx
    
    // Send gain_vector to the MLHD
    ml_SetGainVecAxes(*device_handler, gainset_type, gain_vector);
} // set_internal_gains MaglevSetInternalGains

// Set the Maglev's internal controller gains to default values
//  Only affects the BOUNDARY, LOCKED, and CONSTRAINED gain types.
void MaglevControl::set_other_internal_gains(bool turn_on)
{
    // Iterate over each of the approved gain types
    for (int gain_type = 1; gain_type <= 3; gain_type++)
    {
        // Iterate to assign each direction's gain values
        for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
        {
            if (turn_on)
            {
                // Assign Proportional, Integral, and Derivative gains
                //      in this direction
                gain_vector.values[dirIdx].p  = default_gains_other[dirIdx][0];
                gain_vector.values[dirIdx].i  = default_gains_other[dirIdx][1];
                gain_vector.values[dirIdx].d  = default_gains_other[dirIdx][2];
                if (gain_type == ML_GAINSET_TYPE_LOCKED)
                {
                        // Set feed-forward gains for the LOCKED gain types
                        gain_vector.values[dirIdx].ff = default_gains_other[dirIdx][3];
                }
                else
                {
                        // Do not set feed-forward gains for the CONSTRAINED/BOUNDARY gain types
                        gain_vector.values[dirIdx].ff = 0.0;
                }
            }
            else
            {
                // Set all values to zero
                gain_vector.values[dirIdx].p  = 0.0;
                gain_vector.values[dirIdx].i  = 0.0;
                gain_vector.values[dirIdx].d  = 0.0;
                gain_vector.values[dirIdx].ff = 0.0;
            }
        } // dirIdx
        
        // Send the gains to the Maglev
        ml_SetGainVecAxes(*device_handler, (ml_gainset_type_t)gain_type, gain_vector);
    } // gain_type
} // set_other_internal_gains

// Whether the MLHD is using an external controller (true) or the internal PD controller (false)
bool MaglevControl::is_using_external_control()
{
    return using_external_control;
} // is_using_external_control

// Whether the MLHD is in transition between an external controller (true) or in a set state (false)
bool MaglevControl::is_in_transition()
{
    return in_transition;
} // is_in_transition

// Set the gains for the MLHD during a transition (on or off) to one of our controllers
void MaglevControl::transition_gains()
{
    // Calculate count relative to starting count
    unsigned long long int counter = controller_counter - start_counter;
    if (counter >= 0)
    {
        // Only change values every 10 iterations, to limit the required communication with the MLHD
        if ((counter % 10) == 0)
        {
            // Calculate the proportionality constant for linear interpolation
            double interpolation_constant;
            if (counter < 1)
            {
                // Before the transition, always set to zero
                interpolation_constant = 0.0;
            }
            else if (counter >= NUM_TRANSITIONS)
            {
                // Beyond the transition, always set to one
                interpolation_constant = 1.0;
            }
            else
            {
                // During the transition, calculate a linear transition
                interpolation_constant = ((float)counter) / NUM_TRANSITIONS;
            }
            
            for(int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
            {
                // For each gain (Kp, Ki, Kv/d, Kf)
                for(int gainIdx=0; gainIdx<4; gainIdx++)
                {
                    current_gains_internal[dirIdx][gainIdx] = starting_gains_internal[dirIdx][gainIdx] + (desired_gains_internal[dirIdx][gainIdx]-starting_gains_internal[dirIdx][gainIdx]) * interpolation_constant;
                    current_gains_position[dirIdx][gainIdx] = starting_gains_position[dirIdx][gainIdx] + (desired_gains_position[dirIdx][gainIdx]-starting_gains_position[dirIdx][gainIdx]) * interpolation_constant;
                    current_gains_force[dirIdx][gainIdx] = starting_gains_force[dirIdx][gainIdx] + (desired_gains_force[dirIdx][gainIdx]-starting_gains_force[dirIdx][gainIdx]) * interpolation_constant;
                } // gainIdx
            } // dirIdx
            
            // Set the internal gains
            set_internal_gains();
        }
        
        // If at the end of the transition, stop
        if (counter == NUM_TRANSITIONS)
        {
            // Completely start/stop the force controller
            if (start_counter == 0)
            {
                // Message to the user
                std::cout << "\n**********************************************\n";
                std::cout << "The external controller is completely engaged!\n";
                std::cout << "**********************************************\n";
            }
            else
            {
                // Completely stop the force controller
                std::cout << "\n**********************************************\n";
                std::cout << "The external controller has stopped!\n";
                std::cout << "**********************************************\n";
                using_external_control = false;
            }
            in_transition = false;
        }
    }
    
    return;
} // transition_gains

// Start the external controller
void MaglevControl::start_external_controller()
{
    // Signal that external controller is running
    using_external_control = true;
    in_transition = true;
    
    // Reset the controller counter and the start counter
    controller_counter = 0;
    start_counter = 0;
    
    // Retrieve the current internal gains from the MLHD
    update_internal_gains();
    
    // Set the starting gains for interpolation
    for(int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
    {
        // Set the gain values for each type
        for(int gainIdx=0; gainIdx<4; gainIdx++)
        {
            // Set the desired & starting gains for all three types
            desired_gains_internal[dirIdx][gainIdx] = 0.0;
            starting_gains_internal[dirIdx][gainIdx] = current_gains_internal[dirIdx][gainIdx];
            
            // Set our controller gains based on whether the force controller is
            //  in use in each direction
            // Alternatively, we could set all gains, no matter the force
            //  controller.
            if (force_control_directions[dirIdx])
            {
                desired_gains_position[dirIdx][gainIdx] = 0.0;
                starting_gains_position[dirIdx][gainIdx] = 0.0;
                
                desired_gains_force[dirIdx][gainIdx] = file_gains_force[dirIdx][gainIdx];
                starting_gains_force[dirIdx][gainIdx] = current_gains_force[dirIdx][gainIdx];
            }
            else
            {
                desired_gains_position[dirIdx][gainIdx] = file_gains_position[dirIdx][gainIdx];
                starting_gains_position[dirIdx][gainIdx] = current_gains_position[dirIdx][gainIdx];
                
                desired_gains_force[dirIdx][gainIdx] = 0.0;
                starting_gains_force[dirIdx][gainIdx] = 0.0;
            }
        } // gainIdx
    } // dirIdx
    
    // Turn on the boundary, locked and constrained gains
    set_other_internal_gains(false);
} // start_external_controller

// Stop the external controller
void MaglevControl::stop_external_controller()
{
    // Set the start_counter to be the current value
    start_counter = controller_counter;
    in_transition = true;
    
    // Retrieve the current internal gains from the MLHD
    update_internal_gains();
    
    // Set the starting gains for interpolation
    for(int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
    {
        // Set the gain values for each type
        for(int gainIdx=0; gainIdx<4; gainIdx++)
        {
            // Set the desired & starting gains for all three types
            desired_gains_internal[dirIdx][gainIdx] = default_gains_internal[dirIdx][gainIdx];
            starting_gains_internal[dirIdx][gainIdx] = current_gains_internal[dirIdx][gainIdx];
            
            if (force_control_directions[dirIdx])
            {
                desired_gains_position[dirIdx][gainIdx] = 0.0;
                starting_gains_position[dirIdx][gainIdx] = 0.0;
                
                desired_gains_force[dirIdx][gainIdx] = 0.0;
                starting_gains_force[dirIdx][gainIdx] = current_gains_force[dirIdx][gainIdx];
            }
            else
            {
                desired_gains_position[dirIdx][gainIdx] = 0.0;
                starting_gains_position[dirIdx][gainIdx] = current_gains_position[dirIdx][gainIdx];
                
                desired_gains_force[dirIdx][gainIdx] = 0.0;
                starting_gains_force[dirIdx][gainIdx] = 0.0;
            }
        } // gainIdx
    } // dirIdx
    
    // Turn on the boundary, locked and constrained gains
    set_other_internal_gains(true);
} // stop_external_controller

// Set the save file name
int MaglevControl::set_save_file_name(const std::string file_name)
{
    // Verify that the file can be created
    if (FILE *file = fopen(file_name.c_str(), "w"))
    {
        fclose(file);
	    save_file_name.assign(file_name);
	    return UTAH_NO_ERROR;
    }
    else
    {
        std::cout << "Output file cannot be created! (" << file_name << ")\n";
        return UTAH_BAD_FILE_NAME;
    }
} // set_save_file_name

#include <stdio.h>
// Read the Maglev gains from a data file
void MaglevControl::read_file_gains()
{
    // Verify that the file exists
    FILE *pFile;
    if (!(pFile = fopen(gain_file_name.c_str(), "r")))
    {
        // If we couldn't read the gain file, then just use the defaults in the maglev_parameters.h file
        for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        {
            for (int gainIdx = 0; gainIdx < 3; gainIdx++)
            {
                file_gains_position[dirIdx][gainIdx] = default_gains_position[dirIdx][gainIdx];
                file_gains_force[dirIdx][gainIdx] = default_gains_force[dirIdx][gainIdx];
            } // gainIdx
            file_gains_position[dirIdx][3] = default_gains_position[dirIdx][3];
        } // dirIdx
        
        // Get the default force control directions
        for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        {
            force_control_directions[dirIdx] = default_force_control_directions[dirIdx];
        } // dirIdx
    }
    else
    {
        // Read the force gains from the file
        for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        {
            float a;
            for (int gainIdx = 0; gainIdx < 4; gainIdx++)
            {
                fscanf(pFile, "%f", &a);
                file_gains_position[dirIdx][gainIdx] = a;
            } // gainIdx
            for (int gainIdx = 0; gainIdx < 3; gainIdx++)
            {
                fscanf(pFile, "%f", &a);
                file_gains_force[dirIdx][gainIdx] = a;
            } // gainIdx
        } // dirIdx
        
        // Read the line containing the "directions to control" information
        for (int dirIdx = 0; dirIdx < DATA_SIZE; dirIdx++)
        {
            int a;
            fscanf(pFile, "%d", &a);
            force_control_directions[dirIdx] = (a == 1);
            /*if (a == 0)
            {
                force_control_directions[dirIdx] = false;
            }
            else
            {
                force_control_directions[dirIdx] = true;
            } // */
        } // dirIdx
        
        // Close the file and announce completion
        fclose (pFile);
        return;
    }
}//maglevInitGains

void MaglevControl::print_gain_matrices()
{
    // Make sure the current_gains_internal matrix is up-to-date
    update_internal_gains();
    
    // Print a header for the gain table
    printf("Dir||  Int Kp  |  Int Ki  |  Int Kv  |  Int Kff ||  F2P Kp  |  F2P Ki  |  F2P Kv  |  F2P Kff || F2F Kp| F2F Ki| F2F Kv| F2F Kf|\n");
    printf("---++----------+----------+----------+----------++----------+----------+----------+----------++-------+-------+-------+-------+\n");
    
    // Iterate through each direction and print each gain matrix
    for (int dirIdx=0; dirIdx<DATA_SIZE; dirIdx++)
    {
        // Print the direction
        printf(" %d ||", dirIdx);
        
        // Print the internal position controller gains
        for (int gainIdx=0; gainIdx<4; gainIdx++)
        {
            printf(" %8.2f |", current_gains_internal[dirIdx][gainIdx]);
        }
        
        // Print the external position controller gains
        for (int gainIdx=0; gainIdx<4; gainIdx++)
        {
            printf("| %8.2f ", current_gains_position[dirIdx][gainIdx]);
        }
        
        // Print the force controller gains
        printf("|");
        for (int gainIdx=0; gainIdx<4; gainIdx++)
        {
            printf("| %5.2f ", current_gains_force[dirIdx][gainIdx]);
        }
        printf("|\n");
    }
} // print_gain_matrices

