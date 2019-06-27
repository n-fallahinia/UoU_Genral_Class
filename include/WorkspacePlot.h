// Class definition for WorkspacePlot class
// 
// Written by Thomas R. Grieve
// 24 June 2014
// University of Utah
// Biorobotics Lab

#ifndef WORKSPACEPLOT_H
#define WORKSPACEPLOT_H

#include <FL/Fl_Gl_Window.H>

#include <vector>

// Still need to add a bar on the side to better indicate the z-position

class WorkspacePlot : public Fl_Gl_Window
{
    private:
        /////////////////////////////////////////////////
        // Variables
        
        // Click-and-drag variables
        int click_x, click_y;
        double vx, vy, vz;
        
        // Scaling values for position and rotation
        float scale_position;
        
        // Width and height scaling variables
        int window_width, axis_end;
        
        // Font variables
        void *font;
        
        // Position (x,y,z) of the current axes (axis) and the base axes (base)
        float axis_pose[3];
        float base_pose[3];
        
        // Colors for each region and the base axes
        float pos_axis_color[3];
        float neg_axis_color[3];
        float zer_axis_color[3];
        float base_color[3];
        
        // Line width of the current axes (axis) and the base axes (base)
        float axis_size;
        float base_size;
        
        /////////////////////////////////////////////////
        // Functions
        
        // Draw the axes
        void draw3DAxis();
        
        // Draw a single axis
        void drawAxes(float pose[3], float color[3], float size, bool base_frame);
        
    public:
        /////////////////////////////////////////////////
        // Variables
        
        /////////////////////////////////////////////////
        // Functions
        
        // Constructor
        WorkspacePlot(int x, int y, int w, int h, const char *l=0);
        
        // Draw the window
        void draw();
        
        // Update the axis values
        void set3DAxis(double pose[3]);
        void set3DAxis(float pose[3]);
        void set3DAxis(std::vector<double> pose);
        void set3DAxis(std::vector<float> pose);
        void set_axis(double pose, int axisIdx);
        void set_axis(float pose, int axisIdx);
        
        // Retrieve the position offset values
        //void get_offset(double position[3]) const;
        
        // Handling idle-time and other events
        //bool idle();
        //int handle(int event);
        //bool erase_box();
        
};
#endif

