//
// "$Id: BarPlot.h 5519 2006-10-11 03:12:15Z mike $"
//
// BarPlot class definitions
// 
// Created by Thomas R. Grieve
// 12 June 2014
// University of Utah
// Biorobotics Lab
// 
// Based on CubeView for the Fast Light Tool Kit (FLTK).
//
// Copyright 1998-2005 by Bill Spitzak and others.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA.
//
// Please report all bugs and problems on the following page:
//
//     http://www.fltk.org/str.php
//

#ifndef BARPLOT_H
#define BARPLOT_H 1

#include <FL/Fl_Gl_Window.H>

#include <vector>
#include <string>

class BarPlot : public Fl_Gl_Window
{
    public:
        //////////////////////////////////////////
        // Variables
        
        // Determines whether the display is active or not
        bool flag_force_display;
        
        ////////////////////////////////////////////
        // Functions
        
        BarPlot(int x,int y,int w,int h,const char *l=0);
        
        // Set the height of a given bar
        void set_height(int dirIdx, double height);
        
        // Set the height of all bars
        void set_heights(double* heights);
        
        // The widget class draw() override.
        void draw();    
        
    private:
        //////////////////////////////////////////
        // Variables
        
        // Height of each bar
        std::vector<double> bar_height;
        
        // Names of force directions
        std::vector<std::string> force_names;
        std::vector<int> force_dirs;
        
        // Title, y-label and x-label
        char title[200];
        char y_left[200];
        char y_right[200];
        char xlabel[200];
        
        // Ranges to display
        float min_left, max_left, min_right, max_right;
        float scale_left, scale_right;
        
        // Number of bars to display
        int num_bars;
        
        // Bar width
        float bar_width;
        
        // Slope of the bar function
        double bar_slope;
        
        // The barIdx of the halfway point
        double half_bar;
        
        // Maximum bar position
        double max_bar, mid_bar;
        
        // Desired bar level, used to draw a line across the plot
        float desired_level;
        
        // Coordinates for the 8 vertices of a unit cube
        float boxv0[3];float boxv1[3];
        float boxv2[3];float boxv3[3];
        float boxv4[3];float boxv5[3];
        float boxv6[3];float boxv7[3];
        
        // Display font properties
        Fl_Font font_type;
        Fl_Fontsize font_size;
        
        //////////////////////////////////////////
        // Functions
        
        //  Draw the cube boundaries
        void drawCube();
        void draw_title();
        void draw_ylabels();
        void draw_force_dir(double height, float x_offset, char* my_label);
        
};
#endif

//
// End of "$Id: BarPlot.h 5519 2006-10-11 03:12:15Z mike $".
//
