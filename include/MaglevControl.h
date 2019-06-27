// Header file for MLHD interaction functions

#ifndef MAGLEV_FUNCTIONS_H
#define MAGLEV_FUNCTIONS_H

// Included files
#include "ForceSensor.h"

#include "ml_api.h"

#include <string>

class MaglevControl
{
    private:
        /////////////////////////////////////////////////
        // Variables
        
        // Object for communicating with the MLHD
        ml_device_handle_t* device_handler;
        
        // Defines a vector to read the internal gains
        ml_gain_vec_t gain_vector;
        
        // Type of gains to read
        ml_gainset_type_t gainset_type;
        
        // Current fault condition
        ml_fault_t current_fault;
        
        // MLHD wrench estimate
        ml_forces_t maglev_forces;
        ml_temps_t maglev_temperatures;
        ml_currents_t maglev_currents;
        
        // Arrays of current gains to use in controllers
        double current_gains_force[6][4];
        double current_gains_position[6][4];
        double current_gains_internal[6][4];
        
        // Arrays of gains to achieve after transitions
        double desired_gains_force[6][4];
        double desired_gains_position[6][4];
        double desired_gains_internal[6][4];
        
        // Arrays of gains to start with for the transition
        double starting_gains_force[6][4];
        double starting_gains_position[6][4];
        double starting_gains_internal[6][4];
        
        // Arrays of gains stored in the file
        double file_gains_force[6][4];
        double file_gains_position[6][4];
        
        // Array of directions for use in force control
        bool force_control_directions[6];
        
        // Array of desired position
        double desired_position[6];
        double desired_force[6];
        
        // Array of current position
        double current_position[6];
        
        // Filter constant and proportionality constant for low-pass filter
        double alpha, tau;
        
        // Control frequency for MLHD
        float control_freq;
        
        // Counter tracking the number of times the control loop has run
        unsigned long long int controller_counter;
        unsigned long long int start_counter;
        
        // Booleans indicating states
        bool using_external_control;
        bool in_transition;
        
        // Controller gain file name
        std::string gain_file_name;
        
        // Output file name
        std::string save_file_name;
        
        /////////////////////////////////////////////////
        // Functions
        
        // Update the current estimate of the position
        void update_position();
        
        // Update the current estimate of the forces, temperatures and currents
        void get_force();
        void get_temperature();
        void get_current();
        
        // Update the current matrix of internal gains
        void update_internal_gains();
        
        // Determine whether the flotor is outside the workspace boundary
        bool outside_boundary();
        
        // Read the controller gains from the gain file
        void read_file_gains();
        
    public:
        /////////////////////////////////////////////////
        // Variables
        
        // Determines whether or not to follow the desired trajectory
        bool use_trajectory;
        
        /////////////////////////////////////////////////
        // Functions
        
        // Constructor/Destructor definitions
        MaglevControl();
        ~MaglevControl();
        
        // Functions to retrieve states
        bool is_using_external_control();
        bool is_in_transition();
        
        // Calculate the minimum value of an array (of positions?)
        inline double minValue(double my_array[6]);
        inline float minValue(ml_position_t my_position);
        
        // Retrieve the position of the flotor in a 6-vector
        void get_position(double * position);
        
        // Retrieve the position in a single direction
        double get_position(int dirIdx);
        
        // Connect to/Disconnect from the MLHD
        bool connect();
        void turn_off();
        
        // Set the desired position vector for the class
        void set_desired_position(double position[6]);
        
        // Set the desired wrench vector for the class
        void set_desired_forces(double desired_wrench[6]);
        
        // Set the desired position for the MLHD internal position controller to
        //  be the same as the desired position for the class.
        //  NOTE:  This function does nothing if the internal position
        //  controller is turned off!
        void update_internal_desired_position();
        
        // Retrieve the internal gain from the current_internal_gain matrix
        double get_internal_gain(int dirIdx, int gainIdx);
        
        // Set the internal gains using the current_internal_gain matrix
        void set_internal_gains();
        
        // Update the internal controller gains
        //  If turn_on is true, sets to the default values
        //  If false, sets to 0.0
        //  Only applies to the BOUNDARY, LOCKED, and CONSTRAINED gain types
        void set_other_internal_gains(bool turn_on);
        
        // Set the internal gains and our gains to a transitional value based on
        //  the NUM_TRANSITIONS and the current counter value.  If count_up is
        //  true, our controller will ramp-on while the internal controller
        //  ramps off.  Otherwise, the internal controller will ramp-on while
        //  ours ramps off.
        void transition_gains();
        
        // Send a wrench command to the MLHD.  pid_force must be a 6-vector.
        void set_forces(double * pid_force);
        
        // Print the entire set of gains (internal, position, and force) to stdout.
        void print_gain_matrices();
        
        // Start & stop our external controller
        void start_external_controller();
        void stop_external_controller();
        
        // Set the save file name
        int set_save_file_name(std::string file_name);
        
        /////////////////////////////////////////////////
        // Controller functions
        //  Everything below here must be defined in an external file.
        
        // Position controller
        void PositionController(double current_time);
        
        // Hybrid position-force controller
        void HybridController(double current_time, Reading current_force);
        
        // Digital position controller
        void DigitalController(double current_time);
        
        // Saves the data to the file force_save_file_name
        void save_force_data(char* force_save_file_name);
};

#endif
