//This is a class to handle force sensor
//It start a force sensor, read it with a certain of frequency
//The reading is stored in a queue.
#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include <comedilib.h>

// Define a structure to hold a single force reading
struct Reading
{
    double raw_force_signal[6];
    double force[6];
};

// Define the class to hold all information relating to the force sensor
class ForceSensor
{
    public:
        /////////////////////////////////////////////////
        // Variables
        
        Reading curr_force;
        
        /////////////////////////////////////////////////
        // Functions
        
        // Constructor/Destructor
        ForceSensor();
        ~ForceSensor();
        
        // Indicates that the sensor is turned on
        bool enabled();
        
        // Retrieve a new force reading
        void read();
        
        // Initialize the force sensor
        bool connect();
        
    private:
        /////////////////////////////////////////////////
        // Variables
        
        // Handle for interacting with the Sensoray 626 card
        comedi_t *device;
        
        // References to the on-card devices for input and output
        int analog_input, analog_output;
        
        // Number of analog inputs and outputs on the card
        int num_inputs, num_outputs;
        
        // Channel range used to read the force sensor
        int channel_start, channel_end;
        
        // Name of the Sensoray 626 card according to the OS
        char device_name[20];
        
        // Stores the 'tare' for the force
        double force_tare[6];
        
        // Indicates the sensor is turned on
        bool sensor_on;
        
        /////////////////////////////////////////////////
        // Functions
        
        // Calculate the 'tare'
        void calculate_tare();
        
        // Read the raw voltage signals from the Sensoray
        inline void read_voltage();
        
        // Convert the voltage reading to a force reading
        inline void convert_voltage();
        
        // Apply the 'tare' to the input_force
        inline void apply_tare();
};
#endif
