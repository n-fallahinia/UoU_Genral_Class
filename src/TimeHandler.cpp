// Function definitions for the TimeHandler class

#include "TimeHandler.h"

#include <sched.h>

// Constructor
TimeHandler::TimeHandler()
{
    // Set up the one-second timespec structure
    one_second.tv_sec = 1;
    one_second.tv_nsec = 0L;
    
    // Define the number of nanoseconds in one second
    nanoseconds = 1000000000;
    
    // Define the CPU core to be used for tracking the time
    my_cpu = 3;
    
    // Determine the number of clicks per second
    current_clicks = clicks_per_second();
    
    // Begin timing from the current time
    time_start = 0.0;
    time_start = current_time();
}

// Destructor
TimeHandler::~TimeHandler()
{
}

// Use HPET to get the current time
inline unsigned long long int TimeHandler::hpet_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    
    //return output_value;
    return ((unsigned long long int)ts.tv_sec * nanoseconds + (unsigned long long int)ts.tv_nsec);
} // hpet_time

// Gets the number of clicks per second.
//  Also binds the process to the CPU core.
inline long long int TimeHandler::clicks_per_second()
{
    // Bind the process to my_cpu
    cpu_set_t cpuMask;
    CPU_ZERO(&cpuMask);
    CPU_SET(my_cpu, &cpuMask);
    sched_setaffinity(0, sizeof(cpuMask), &cpuMask);
    
    // Measure number of clicks during 1 second
    unsigned long long int curr_time = hpet_time();
    nanosleep(&one_second, (struct timespec *)NULL);
    
    // Return the difference
    return (long long int)(hpet_time() - curr_time);
} // clicks_per_second

// Resets the starting time to the current time
void TimeHandler::reset_timer()
{
    // Start the timer over at zero
    time_start = 0.0;
    time_start = current_time();
} // reset_timer

// Retrieves the current time (in seconds) with the starting offset time removed
double TimeHandler::current_time()
{
    return (((double)hpet_time()) / ((double)current_clicks) - time_start);
} // current_time

