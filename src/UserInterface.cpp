// generated by Fast Light User Interface Designer (fluid) version 1.0302

#include "UserInterface.h"
#include "Constants.h"

#include <FL/Fl.H>
#include <FL/fl_ask.H>

// Standard Constructor
UserInterface::UserInterface()
{
    my_force = new ForceSensor();
    my_maglev = new MaglevControl();
    my_timer = new TimeHandler();
}

// Destructor
UserInterface::~UserInterface()
{
    delete my_force;
    delete my_maglev;
    delete my_timer;
}

void UserInterface::cb_btnStartController_i(Fl_Button* o, void*)
{
    if (btnStartController->value())
    {
        // Display a zero trajectory
        float desired_position[] = {0.0, 0.0};
        box_time->readDesiredData(desired_position);
        desired_position[1] = ((STROKE_LENGTH / STROKE_VELOCITY) * NUM_STROKES + INITIAL_TIME) / 0.001;
        box_time->readDesiredData(desired_position);
        
        // Start the MLHD
        if (!my_maglev->connect())
	    {
            fl_alert("Incorrect MLHD initialization!");
	        my_maglev->turn_off();
	        return;
        }
        
        // Redraw the position box
        box_time->redraw();
        
        // Start our external controller
        my_maglev->start_external_controller();
        
        // Reset the timer
        my_timer->reset_timer();
        box_time->clear_data();
        
        // Set up timer to display positions
        Fl::add_timeout(1.0/POSITION_FREQUENCY, display_position_reading);
        
        // Change the label of this button to "Start Controller"
        btnStartController->label("Stop Controller");
        
        // Activate the "Begin Experiment" button
        btnTest->activate();
    }
    else
    {
        // Deactivate the "Begin Experiment" button
        btnTest->deactivate();
        
        // Stop our external controller
        my_maglev->stop_external_controller();
        while (my_maglev->is_in_transition());
        
        // Stop the timer
        Fl::remove_timeout(display_position_reading);
        
        // Turn off the MLHD
        my_maglev->turn_off();
        
        // Clear the display
        box_time->clear_data();
        box_time->clear_desired();
        
        // Change the label of this button to "Start Controller"
        btnStartController->label("Start Controller");
    }
}
void UserInterface::cb_btnStartController(Fl_Button* o, void* v) {
  ((UserInterface*)(o->parent()->parent()->user_data()))->cb_btnStartController_i(o,v);
}

void UserInterface::cb_btnTest_i(Fl_Button*, void*) {
    // Start the experiment
    if (btnTest->value())
    {
        // Deactivate the "Stop Controller" button
        btnStartController->deactivate();
		cmbTongue->deactivate();
		cmbProduct->deactivate();
		cmbHydration->deactivate();
        
		// Set the file name
		char save_file[500];
		sprintf(save_file, "data_%02d_%02d_%02d.txt", cmbTongue->value(), cmbHydration->value(), cmbProduct->value());
        my_maglev->set_save_file_name(save_file);
		
        // Change the label of this button to "Stop Experiment"
        btnTest->label("Stop Experiment");
        
        // Read the desired position trajectory
        read_trajectory();
        box_time->redraw();
        
        // Reset the timer
        my_timer->reset_timer();
        box_time->clear_data();
        
        // Set the MLHD to use the trajectory
        my_maglev->use_trajectory = true;
    }
    else
    {
        // Set the MLHD to stop using the trajectory
        my_maglev->use_trajectory = false;
        
        // Change the label of this button to "Begin Experiment"
        btnTest->label("Begin Experiment");
        
        // Activate the "Stop Controller" button
        btnStartController->activate();
		cmbTongue->activate();
		cmbProduct->activate();
		cmbHydration->activate();
    }
}
void UserInterface::cb_btnTest(Fl_Button* o, void* v) {
  ((UserInterface*)(o->parent()->parent()->user_data()))->cb_btnTest_i(o,v);
}

void UserInterface::cb_btnForceSensor_i(Fl_Button*, void*) {
    // Start the Force Sensor
    if (btnForceSensor->value())
    {
        // Start the Force Sensor
        if (!my_force->connect())
	     {
	        fl_alert("Could not start force sensor!");
	        return;
        }
        bar_force->redraw();
        
        // Set up timer to display forces
        Fl::add_timeout(1.0/FORCE_FREQUENCY, display_force_reading);
        
        // Activate the "Start Controller" button
        btnStartController->activate();
        
        // Deactivate this button
        btnForceSensor->deactivate();
    }
}
void UserInterface::cb_btnForceSensor(Fl_Button* o, void* v) {
  ((UserInterface*)(o->parent()->parent()->user_data()))->cb_btnForceSensor_i(o,v);
}

void UserInterface::make_window() {
  { main_window = new Fl_Double_Window(100, 100, 630, 630, "University of Utah MLHD Interface");
    main_window->user_data((void*)(this));
    { box_time = new TimePlot(10, 10, 300, 300);
      box_time->box(FL_BORDER_BOX);
      box_time->color(FL_BACKGROUND_COLOR);
      box_time->selection_color(FL_BACKGROUND_COLOR);
      box_time->labeltype(FL_NORMAL_LABEL);
      box_time->labelfont(0);
      box_time->labelsize(14);
      box_time->labelcolor(FL_FOREGROUND_COLOR);
      box_time->align(Fl_Align(FL_ALIGN_CENTER));
      box_time->when(FL_WHEN_RELEASE);
    } // TimePlot* box_time
    { bar_force = new BarPlot(10, 320, 300, 300);
      bar_force->box(FL_BORDER_BOX);
      bar_force->color(FL_BACKGROUND_COLOR);
      bar_force->selection_color(FL_BACKGROUND_COLOR);
      bar_force->labeltype(FL_NORMAL_LABEL);
      bar_force->labelfont(0);
      bar_force->labelsize(14);
      bar_force->labelcolor(FL_FOREGROUND_COLOR);
      bar_force->align(Fl_Align(FL_ALIGN_CENTER));
      bar_force->when(FL_WHEN_RELEASE);
    } // BarPlot* bar_force
    { my_workspace = new WorkspacePlot(320, 320, 300, 300);
      my_workspace->box(FL_BORDER_BOX);
      my_workspace->color(FL_BACKGROUND_COLOR);
      my_workspace->selection_color(FL_BACKGROUND_COLOR);
      my_workspace->labeltype(FL_NORMAL_LABEL);
      my_workspace->labelfont(0);
      my_workspace->labelsize(14);
      my_workspace->labelcolor(FL_FOREGROUND_COLOR);
      my_workspace->align(Fl_Align(FL_ALIGN_CENTER));
      my_workspace->when(FL_WHEN_RELEASE);
    } // WorkspacePlot* my_workspace
    { group_options = new Fl_Group(320, 10, 300, 300);
      group_options->box(FL_ENGRAVED_BOX);
      group_options->align(Fl_Align(FL_ALIGN_TOP_LEFT));
      { cmbTongue = new Fl_Choice(410, 20, 65, 30, "Subj ID:");
        cmbTongue->down_box(FL_BORDER_BOX);
          // Add an invisible, blank item to the list
          cmbTongue->add("<nothing>",0,0,0,FL_MENU_INVISIBLE);
          // Add each finger to the list
          char temp[2];
          for (int i=1; i<=10; i++)
          {
            sprintf(temp, "%02d", i);
            cmbTongue->add(temp);
          }
          cmbTongue->value(1);
        // Load tongue data
      } // Fl_Choice* cmbTongue
      { cmbProduct = new Fl_Choice(410, 55, 150, 30, "Force Grid");
        cmbProduct->down_box(FL_BORDER_BOX);
        cmbProduct->add("Cart.");
        cmbProduct->add("Spher.");
        // Load product data
      } // Fl_Choice* cmbProduct
      { cmbHydration = new Fl_Choice(410, 90, 150, 30, "Test");
        cmbHydration->down_box(FL_BORDER_BOX);
        cmbHydration->add("1");
        cmbHydration->add("2");
        // Load hydration data
      } // Fl_Choice* cmbHydration
      { btnForceSensor = new Fl_Button(410, 200, 150, 30, "Start Force Sensor");
        btnForceSensor->type(1);
        btnForceSensor->callback((Fl_Callback*)cb_btnForceSensor);
      } // Fl_Button* btnForceSensor
      { btnStartController = new Fl_Button(410, 235, 150, 30, "Start Controller");
        btnStartController->type(1);
        btnStartController->deactivate();
        btnStartController->callback((Fl_Callback*)cb_btnStartController);
      } // Fl_Button* btnStartController
      { btnTest = new Fl_Button(410, 270, 150, 30, "Begin Experiment");
        btnTest->type(1);
        btnTest->deactivate();
        btnTest->callback((Fl_Callback*)cb_btnTest);
      } // Fl_Button* btnTest
      group_options->end();
    } // Fl_Group* group_options
    main_window->end();
  } // Fl_Double_Window* main_window
}

void UserInterface::readDesiredData(float* desired_data)
{
    box_time->readDesiredData(desired_data);
}

/**
   Connects to the MLHD, lifts off, and starts the controller.
   Also initializes the Force Sensor.
*/
int UserInterface::start_controller() {
  // Initialize the force sensor
  
  // Start a timer to draw the force readings
  
  // Connect to the MLHD
  
  // Start a timer to draw the position readings
  
  // Lift off
  
  // Start the controller
  
  // If all went well, change the text.  Otherwise, revert the button state.
  
  // If the controller was already running, shut everything down instead.
return 0;
}

/**
   Runs a single tongue-stroking experiment.
*/
int UserInterface::begin_experiment() {
  // Run the trajectory
return 0;
}


