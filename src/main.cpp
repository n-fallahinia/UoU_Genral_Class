#include "main.h"

#include <FL/Fl.H>

#include <iostream>

// Time step to use for regulating the loop
const double time_step = 0.001; // seconds

// Global variable for the user interface
UserInterface* my_window = new UserInterface();

// Main function
int main(int argc, char* argv[])
{
    // Start the window
    my_window->make_window();
    std::cout << "Got here!\n";
    
    // Initialize a desired wrench vector
    double desired_wrench[] = {0.0, 0.0, -0.5, 0.0, 0.0, 0.0};
    my_window->my_maglev->set_desired_forces(desired_wrench);
    
    // Check inputs for flaws
	int i = 0;
	if (Fl::args(argc,argv,i) < argc)
		Fl::fatal(Fl::help);
	
	// Turn on full/true color graphics
	Fl::visual(FL_RGB);
	
	// Display the main window
	my_window->main_window->show(argc,argv);
	return Fl::run();
} // main function

// Read data into TimePlot box
void read_trajectory()
{
    // Clear existing trajectory
    my_window->box_time->clear_desired();
    
    // Determine the amount of time required for the trajectory
    double total_time = (STROKE_LENGTH / STROKE_VELOCITY) * NUM_STROKES + INITIAL_TIME;
    double num_steps = (total_time) / time_step;
    if (num_steps > NUM_DATA)
    {
        std::cout << "Not enough data points for the given trajectory!\n\tNeed to increase NUM_DATA to at least " << num_steps << "!\n";
    }
    num_steps = NUM_DATA;
    
    // Variable to hold current time and desired position
    float desired_position[] = {0.0, 0.0};
    
    // Process each step
    for (int stepIdx = 0; stepIdx < num_steps; stepIdx++)
    {
        // Retrieve the trajectory position based on the time
        desired_position[0] = get_desired_position(desired_position[1]);
        
        // Send the desired position to the TimePlot box
        my_window->box_time->readDesiredData(desired_position);
        
        // Increment the time
        desired_position[1] += time_step;
    } // stepIdx
} // read_trajectory

void display_force_reading(void * v)
{
    if (my_window->my_force->enabled())
    {
        // Send a new force reading to the display box
        my_window->my_force->read();
        my_window->bar_force->set_heights(my_window->my_force->curr_force.force);
        my_window->bar_force->redraw();
        
        // Set up the next iteration of this timer
        Fl::repeat_timeout(1.0/FORCE_FREQUENCY, display_force_reading);
    }
} // display_force_reading

void display_position_reading(void * v)
{
    if (my_window->my_maglev->is_using_external_control())
    {
        // Send a new position reading to the display box
        float actual_position[] = {0.0, 0.0};
        actual_position[0] = my_window->my_maglev->get_position(0);
        actual_position[1] = my_window->my_timer->current_time();
        my_window->box_time->readData(actual_position);
        my_window->box_time->redraw();
        
        // Send the position reading to the workspace plot box
        double current_position[6];
        my_window->my_maglev->get_position(current_position);
        my_window->my_workspace->set3DAxis(current_position);
        my_window->my_workspace->redraw();
        
        // Set up the next iteration of this timer
        Fl::repeat_timeout(1.0/POSITION_FREQUENCY, display_position_reading);
    }
} // display_position_reading

int tick_callback_handler( ml_device_handle_t maglev_handle,  ml_position_t *maglev_position )
{
    // Get a new force sensor reading
    double current_time = my_window->my_timer->current_time();
    my_window->my_force->read();
    
    // Verify that the Maglev force controller is running
    if (my_window->my_maglev->is_using_external_control())
    {
        // Use the desired trajectory
        if (my_window->my_maglev->use_trajectory)
        {
            double next_position[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            next_position[0] = get_desired_position(current_time);
            my_window->my_maglev->set_desired_position(next_position);
        }
        
        // Transition the gains, if needed
        my_window->my_maglev->transition_gains();
        
        // Run the Maglev Controller
        //my_window->my_maglev->DigitalController(current_time);
        my_window->my_maglev->PositionController(current_time);
        //my_window->my_maglev->HybridController(current_time, my_window->my_force->curr_force);
    }
    
    // Set the return value
    return 0;
}

