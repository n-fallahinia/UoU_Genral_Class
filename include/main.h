
#ifndef MAIN_H
#define MAIN_H

#include "UserInterface.h"
#include "Constants.h"

#include "ml_api.h"

// Function definitions
double get_desired_position(double current_time);
void read_trajectory();
void display_force_reading(void * v);
void display_position_reading(void * v);
int tick_callback_handler(ml_device_handle_t maglev_handle,  ml_position_t *maglev_position);

#endif

