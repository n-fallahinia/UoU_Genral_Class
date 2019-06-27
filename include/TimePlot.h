// Plot 2D data with OpenGL
// It allows subplots in one window
// It allows multiple curves in one plot
// For each plot, the input is x, y[n], data_num, data_dim, title, xlable, ylabel
// It has options to show connected lines, dots only, or dots and lines

#ifndef TIMEPLOT_H
#define TIMEPLOT_H

#include <FL/Fl_Gl_Window.H>

#include <queue>

// Set up a data structure to hold a single reading
struct ReadingDisplay
{
    double data;
    double data_x;
};

class TimePlot : public Fl_Gl_Window
{
    public:
        ///////////////////////////////////////////////////////////////////////////////////////////
        // Variables
        
        // Number of points actually in the queue, does not appear to be initialized
        int data_size, desired_size;
        
        // Maximum number of data points allowed
        //unsigned int max_size;
        
        ///////////////////////////////////////////////////////////////////////////////////////////
        // Functions
        
        // Constructor
        TimePlot(int x,int y,int w,int h,const char *l=0);
        
        // Add the data in curr_force to the queue for display
        void readData(float* current_position);
        void readDesiredData(float* desired_position);
        
        // Empty the current data queue & desired data vector
        void clear_data();
        void clear_desired();
        
        // Redraws the widget
        void draw();
        
    private:
        ///////////////////////////////////////////////////////////////////////////////////////////
        // Variables
        
        // Desired data points for plotting
        std::vector<ReadingDisplay> desired_data;
        
        // Actual data points for plotting
        std::queue<ReadingDisplay> data_display;
        
        // Figure size variables, used to determine size of text and lines
        //  Set as fixed numbers
        float width, height;
        
        // Character array containing a string to display before the experiment begins
        char instruction_string[200];
        
        // Stores title, xlabel and ylabel for the figure
        char title[200];
        char xlabel[200];
        char ylabel[200];
        
        // Colors for plotting data
        float desired_color[3];
        float data_color[3];
        float line_color[3];
        
        // Scale and offset values for plotting
        float x_scale, y_scale, x_offset, y_offset;
        
        ///////////////////////////////////////////////////////////////////////////////////////////
        // Functions
        
        // Draws the data
        void drawData(int, int);
        void drawDesired(int, int);
        
        // Draws the title, xlabel and ylabel on the plot
        void drawTitle();
        void drawYLabel();
        void drawXLabel();
        
        // Draws the x-axis
        void drawZeroLines();
        
        // Draws the phrase used before the experiment begins
        void drawInstruction();
};
#endif

