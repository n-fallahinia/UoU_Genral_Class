#include "ForceSensor.h"
#include <cstdlib>
#include <iostream>

// Define the force sensor that is attached to the MLHD
#define FT9555

//Determine the force sensor in use and load the appropriate calibration and gain matrices.
#ifdef FT9556
    // The FT9556 force sensor is in use, load its calibration and gain matrices
    double calib[6][6] = {{  0.00823,  0.15291, -1.20116,-39.78579, -0.93552, 39.73839},
                          {  0.33258, 45.74619, -1.03095,-22.18827,  0.53428,-22.84706},
                          { 22.32658,  0.90127, 22.76112,  0.93060, 22.53465, -0.26441},
                          { -0.64595, -0.57383, 38.12715,  1.64403,-38.94665,  0.81836},
                          {-44.19189, -1.80720, 22.70840,  0.55018, 22.70044,  0.12393},
                          { -0.02791, 22.94270,  0.71725, 22.60032, -0.23238, 23.53918}};
    double gain[6] = {22.7300084098357, 22.7300084098357, 11.7868333238604, 3.64585766892664, 3.64585766892664, 3.1484034675519};
#else
    #ifdef FT9555
        // The FT7151 force sensor is in use; load its calibration and gain matrices
        double calib[6][6] = {{   0.06105,  -0.19672,  -1.79433,  37.70342,   2.06223, -37.52904},
                              {   2.02210, -44.83887,  -1.40611,  21.12528,  -1.40373,  22.29920},
                              { -22.17321,  -0.29397, -22.09094,  -0.75540, -22.46264,  -0.83756},
                              {   0.86343,   0.62380, -37.57811,  -1.73684,  39.28231,   1.42668},
                              {  43.45049,   0.61411, -21.70741,  -0.41397, -23.31255,  -1.47309},
                              {   0.44082, -22.53919,   1.48699, -21.63087,   1.43589, -22.34803}};
        double gain[6] = {22.7300084098357, 22.7300084098357, 11.7868333238604, 3.64585766892664, 3.64585766892664, 3.1484034675519};
    #else
        // These are the default "dummy" calibration and gain matrices to use when a bad force sensor has been specified.
        double calib[6][6] = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        double gain[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    #endif
#endif

// Standard Constructor
ForceSensor::ForceSensor()
{
    // Channels 16-21 are where the force sensor is currently connected
    channel_start = 0;
    channel_end = 5;
    
    // The force sensor is turned off when the sensor is initialized
    sensor_on = false;
    
    // Define the name of the device
    sprintf(device_name,"/dev/comedi0");
} // Standard Constructor for the ForceSensor

// ForceSensor Destructor
ForceSensor::~ForceSensor()
{
    printf("ForceSensor Destructor\n");
    if (sensor_on)
        comedi_close(device);
} // ForceSensor Destructor

// Indicates whether the sensor is turned on
bool ForceSensor::enabled()
{
    return sensor_on;
} // enabled

// Initialize the Sensoray card to read from the force sensor
bool ForceSensor::connect()
{
    // If the force sensor is already on, don't try to start it again
    if (sensor_on)
    {
        printf("The force sensor has already been started\n");
        return true;
    }
    
    // Start the DAC
    device = comedi_open ( device_name );
    if ( !device )
    {
        // If the device did not start, give an error message and quit
        comedi_perror ( device_name );
        printf("Could not open the Sensoray card!\n");
        return false;
    }
    
    // Determine the force sensor offset
    calculate_tare();
    printf("Force sensor tare values: [%5.2f", force_tare[0]);
    for (int temp = 1; temp < 6; temp++)
    {
        printf(" %5.2f", force_tare[temp]);
    }
    printf("]\n");
    
    // Determine the sub-device numbers for the analog input and analog output
    analog_input = comedi_find_subdevice_by_type(device, COMEDI_SUBD_AI, 0);
    // printf("analog input sub_device number: %d \n",analog_input); 
    analog_output = comedi_find_subdevice_by_type(device, COMEDI_SUBD_AO, 0);
    num_inputs = comedi_get_n_channels(device, analog_input);
    num_outputs = comedi_get_n_channels(device, analog_output);

    // Mark that the force sensor is now capable of recording data
    sensor_on = true;
    return true;
} // connect

// Collect & process a new force reading
void ForceSensor::read()
{
    // Read the voltage from the DAC
    read_voltage();
    
    //Convert the voltage to force reading
    convert_voltage();
    
    // Subtract the force offset
    apply_tare();
} // read

// Read the voltage values from the Sensoray card
inline void ForceSensor::read_voltage()
{
    // Declare variables
    int n_chans, chanIdx, forceIdx;
    long int max_data;
    comedi_range *rgn;
    lsampl_t data;
    
    // Open the device
    n_chans = comedi_get_n_channels(device, analog_input);
    // printf("channel number %d \n", n_chans);
    if (channel_end > n_chans)
    {
        printf("The channel range is wrong -- %d \n", n_chans);
        exit(-1);
    }
    
    // Read the force sensor channels
    for (chanIdx = channel_start; chanIdx <= channel_end; ++chanIdx)
    {
        // Calculate the zero-based index of the current channel
        forceIdx = chanIdx - channel_start;
        
        // I am not sure why we need this parameter
        int range = 1;
        
        // Determine the maximum data value of the channel -- Do we need to do this on every sampling?
        max_data = comedi_get_maxdata(device, analog_input, chanIdx);
        
        // Determine the range of the channel -- Do we need to do this on every sampling?
        rgn = comedi_get_range(device, analog_input, chanIdx, range);
        
        // Read the data from the current channel
        comedi_data_read(device, analog_input, chanIdx, range, AREF_GROUND, &data);
        
        // If at the maximum data level, subtract one (Why?)
        if ( data == max_data )
        {
            data = data - 1;
        }
        
        // If at the minimum data level, add one (Why?)
        if (data == 0)
        {
            data = 1;
        }
        // printf("%d\n", data);
        // Convert the data value to a voltage value
        curr_force.raw_force_signal[forceIdx] = comedi_to_phys(data, rgn, max_data);
        //printf("%f\n", curr_force.raw_force_signal[0]);
    } // chanIdx
} // read_voltage

// Convert the voltage reading to a force reading
inline void ForceSensor::convert_voltage()
{
    // Sum over rows (forces)
    for(int forceIdx = 0; forceIdx < 6; forceIdx++)
    {
        // Assign initial value for force
	    curr_force.force[forceIdx] = 0.0;
	    
	    // Sum over columns (voltages)
	    for(int voltageIdx = 0; voltageIdx < 6; voltageIdx++)
	    {
            curr_force.force[forceIdx] += calib[forceIdx][voltageIdx] * curr_force.raw_force_signal[voltageIdx];
	    } // voltageIdx
	    
	    // Divide by gain value
	    curr_force.force[forceIdx] /= gain[forceIdx];
    } // forceIdx
} // convert_voltage

// Apply the 'tare' value to force_curr
inline void ForceSensor::apply_tare()
{
    for (int forceIdx=0; forceIdx < 6; forceIdx++)
    {	
		curr_force.force[forceIdx] -= force_tare[forceIdx];
    } // forceIdx
} // apply_tare

// Calculate the 'tare' values in all directions
void ForceSensor::calculate_tare()
{
    // Number of data points used to estimate the tare
    int force_zero_average_num = 10;
    
    // Set initial sums to zero
    force_tare[0] = force_tare[1] = force_tare[2] = force_tare[3] = force_tare[4] = force_tare[5] = 0.0;
    
    // Process given number of data points
    for (int dataIdx = 0; dataIdx < force_zero_average_num; dataIdx++)
    {
        // Read the voltage
		read_voltage();
		
		// Convert the voltage to force reading
		convert_voltage();
		
		// Store offsets for each channel
		for (int forceIdx = 0; forceIdx < 6; forceIdx++)
		{
            force_tare[forceIdx] += curr_force.force[forceIdx] / ((double)force_zero_average_num);
		} // forceIdx
    } // dataIdx
} // calculate_tare

