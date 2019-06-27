// All the functions that handle the timer

#ifndef TIMEHANDLER_H
#define TIMEHANDLER_H

#include <time.h>

class TimeHandler
{
    private:
        /////////////////////////////////////////////////
        // Variables
        
        // Number of clicks in one second
        long long int current_clicks;
        
        // Number of nanoseconds in one second
        unsigned long long int nanoseconds;
        
        // A timespec structure that will be set up to hold exactly 1 second
        timespec one_second;
        
        // Define the CPU core to use for tracking the time
        int my_cpu;
        
        // Initial time
        double time_start;
        
        /////////////////////////////////////////////////
        // Functions
        
        // Collects the time (in nanoseconds) from the CPU
        inline unsigned long long int hpet_time();
        
        // Determines the number of clicks in one second
        inline long long int clicks_per_second();
        
    public:
        /////////////////////////////////////////////////
        // Variables
        
        /////////////////////////////////////////////////
        // Functions
        
        // Constructors
        TimeHandler();
        ~TimeHandler();
        
        // Retrieve the current time
        double current_time();
        
        // Resets the starting time
        void reset_timer();
};

#endif

