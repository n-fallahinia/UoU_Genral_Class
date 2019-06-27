//
// "$Id: BarPlot.cxx 5519 2006-10-11 03:12:15Z mike $"
//
// BarPlot class implementation
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

// Used to plot sensor force/torque

#include "BarPlot.h"
#include "Constants.h"

#include <FL/Fl.H>
#include <FL/gl.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>

BarPlot::BarPlot(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x,y,w,h,l)
{
    // Set the boxes to height zero
    num_bars = 1;
    bar_height.assign(num_bars, 0.0);
    bar_width = 10.0 / ((float)num_bars);
    
    // Define the force direction names
    //  This is a clunky way to do this, but it allows us to view any
    //  combination of 1-6 forces/torques at once
    if (num_bars > 1)
    {
        force_names.push_back("Fx");
        force_names.push_back("Fy");
        force_dirs.push_back(0);
        force_dirs.push_back(1);
    }
    force_names.push_back("Fz");
    force_dirs.push_back(2);
    if (num_bars > 3)
    {
        force_names.push_back("Tx");
        force_names.push_back("Ty");
        force_names.push_back("Tz");
        force_dirs.push_back(3);
        force_dirs.push_back(4);
        force_dirs.push_back(5);
        
        // Set the maximum and midpoint bar positions
        max_bar = 5.0;
        mid_bar = -0.5;
    }
    else
    {
        // Set the maximum and midpoint bar positions
        max_bar = 4.5;
        mid_bar = -0.5;
    }
    
    // Calculate the halfway point across the bar indices
    half_bar = ((double)num_bars-1.0)/2.0;
    if (num_bars == 1)
    {
        bar_slope = 0.0;
    }
    else
    {
        bar_slope = ((max_bar - mid_bar)/half_bar);
    }
    
    // Define the title, x-label and y-label
    sprintf(title, "%s", "Measured Normal Force");
    sprintf(y_left, "%s", "(N)");
    sprintf(y_right, "%s", "(N-mm)");
    
    // Define scale for display
    min_left = -9.0;
    max_left = 9.0;
    scale_left = 3.0;
    min_right = -300.0;
    max_right = 300.0;
    scale_right = 100.0;
    
    // Define desired bar level
    desired_level = -3.0;
    
    // The cube definition. These are the vertices of a unit cube
    //   centered on the origin.
    boxv0[0] = -0.5; boxv0[1] = -0.5; boxv0[2] = -0.5;
    boxv1[0] =  0.5; boxv1[1] = -0.5; boxv1[2] = -0.5;
    boxv2[0] =  0.5; boxv2[1] =  0.5; boxv2[2] = -0.5;
    boxv3[0] = -0.5; boxv3[1] =  0.5; boxv3[2] = -0.5;
    boxv4[0] = -0.5; boxv4[1] = -0.5; boxv4[2] =  0.5;
    boxv5[0] =  0.5; boxv5[1] = -0.5; boxv5[2] =  0.5;
    boxv6[0] =  0.5; boxv6[1] =  0.5; boxv6[2] =  0.5;
    boxv7[0] = -0.5; boxv7[1] =  0.5; boxv7[2] =  0.5;
    
    // Define the font properties
    font_type = FL_COURIER;
    //font_type = FL_HELVETICA_BOLD;
    font_size = 12;
    
    int barIdx = 0;
    std::cout << force_names[barIdx] << ":" << force_dirs[barIdx] << "\n";
    std::cout << L_SCALE*bar_height[barIdx] << "::" << bar_slope*((double)barIdx - half_bar) << "\n";
} // Standard Constructor

// Set the height of a bar
//  For this application (tongue_force), we are only concerned about negative
//  force, so the sign convention will be switched for display purposes.
void BarPlot::set_height(int dirIdx, double height)
{
    if ((dirIdx < 0) || (dirIdx >= num_bars))
    {
        std::cout << "Attempted to set height for incorrect barIdx! (" << dirIdx << ")\n";
        return;
    }
    bar_height[dirIdx] = height;
} // set_height

// Set the heights of all bars
void BarPlot::set_heights(double* heights)
{
    for (int barIdx = 0; barIdx < num_bars; barIdx++)
    {
        bar_height[barIdx] = heights[force_dirs[barIdx]];
    }
} // set_heights

// Draw the faces of the cube using the boxv[] vertices, using
//  GL_LINE_LOOP for the faces. The color is \#defined by CUBECOLOR.
void BarPlot::drawCube()
{
    /* Draw a colored cube */
#define ALPHA 0.5
    glShadeModel(GL_FLAT);

    glBegin(GL_QUADS);
      glColor4f(0.0, 0.0, 1.0, ALPHA);
      glVertex3fv(boxv0);
      glVertex3fv(boxv1);
      glVertex3fv(boxv2);
      glVertex3fv(boxv3);

      glColor4f(1.0, 1.0, 0.0, ALPHA);
      glVertex3fv(boxv0);
      glVertex3fv(boxv4);
      glVertex3fv(boxv5);
      glVertex3fv(boxv1);

      glColor4f(0.0, 1.0, 1.0, ALPHA);
      glVertex3fv(boxv2);
      glVertex3fv(boxv6);
      glVertex3fv(boxv7);
      glVertex3fv(boxv3);

      glColor4f(1.0, 0.0, 0.0, ALPHA);
      glVertex3fv(boxv4);
      glVertex3fv(boxv5);
      glVertex3fv(boxv6);
      glVertex3fv(boxv7);

      glColor4f(1.0, 0.0, 1.0, ALPHA);
      glVertex3fv(boxv0);
      glVertex3fv(boxv3);
      glVertex3fv(boxv7);
      glVertex3fv(boxv4);

      glColor4f(0.0, 1.0, 0.0, ALPHA);
      glVertex3fv(boxv1);
      glVertex3fv(boxv5);
      glVertex3fv(boxv6);
      glVertex3fv(boxv2);
    glEnd();

    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
      glVertex3fv(boxv0);
      glVertex3fv(boxv1);

      glVertex3fv(boxv1);
      glVertex3fv(boxv2);

      glVertex3fv(boxv2);
      glVertex3fv(boxv3);

      glVertex3fv(boxv3);
      glVertex3fv(boxv0);

      glVertex3fv(boxv4);
      glVertex3fv(boxv5);

      glVertex3fv(boxv5);
      glVertex3fv(boxv6);

      glVertex3fv(boxv6);
      glVertex3fv(boxv7);

      glVertex3fv(boxv7);
      glVertex3fv(boxv4);

      glVertex3fv(boxv0);
      glVertex3fv(boxv4);

      glVertex3fv(boxv1);
      glVertex3fv(boxv5);

      glVertex3fv(boxv2);
      glVertex3fv(boxv6);

      glVertex3fv(boxv3);
      glVertex3fv(boxv7);
    glEnd();
}//drawCube

// Draws the entire area
void BarPlot::draw()
{
    if (!valid())
    {
        glLoadIdentity();
        glViewport(0,0,w(),h());
        glOrtho(-10,10,-10,10,-20050,10000);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Draw the x-axis
    float zero0[] = {L_AXIS, X_AXIS, 0.0};
    float zero1[] = {-L_AXIS, X_AXIS, 0.0};
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
        glVertex3fv(zero0);
        glVertex3fv(zero1);
    glEnd();
    
    // Draw the Title
    draw_title();
    
    // Draw the y-axis label
    draw_ylabels();
    
    // Draw the force box
    for (int barIdx = 0; barIdx < num_bars; barIdx++)
    {
        char p_t[200];
        sprintf(p_t, "%s", force_names[barIdx].c_str());
        if (barIdx < 3)
            draw_force_dir(L_SCALE*bar_height[barIdx], bar_slope*((double)barIdx - half_bar) + mid_bar, p_t);
        else
            draw_force_dir(R_SCALE*bar_height[barIdx], bar_slope*((double)barIdx - half_bar) + mid_bar, p_t);
    } // barIdx
    
    // Draw the desired bar line
    zero0[1] = L_SCALE*desired_level + X_AXIS;
    zero1[1] = L_SCALE*desired_level + X_AXIS;
    glColor3f(0.5, 0.5, 0.5);
    glBegin(GL_LINES);
        glVertex3fv(zero0);
        glVertex3fv(zero1);
    glEnd();
} // draw

// Draws the label for the force direction and the box showing its value
//      (height), at the specified x-position (x_offset)
void BarPlot::draw_force_dir(double height, float x_offset, char* my_label)
{
    // Draw the box for this direction of force
    glPushMatrix();
        glTranslatef(x_offset, X_AXIS, 0.0);
        glTranslatef(0.0, float(height/2.0), 0.0);
        //printf(" %5.2f)\n", height/2.0);
        glScalef(bar_width, float(height), 1.0);
        drawCube();
    glPopMatrix();
    
    // Label this bar
    gl_font(font_type, font_size);
    glColor3f(1.0, 1.0, 1.0);
    glRasterPos3f(x_offset-0.5, X_AXIS-1.0, 0.0);   
    char r_z[] = "  ";
    sprintf(r_z, "%s", my_label);
    gl_draw(my_label, strlen(my_label));
} // draw_force_dir

// Draws the title
void BarPlot::draw_title()
{
    gl_font(font_type, font_size);
    glColor3f(1.0, 1.0, 1.0);
    glRasterPos3f(L_AXIS/2.0, TITLE_POSITION, 0.0);   
    gl_draw(title, strlen(title));
} // draw_title

// Draws the y-labels
void BarPlot::draw_ylabels()
{
	//Label the y-axis
	gl_font(font_type, font_size);
	glColor3f(1.0, 1.0, 1.0);
	
	// Declare the initial value for the string to hold the y-axis label
	char left_label[] = "  0";
    
    // Draw all of the y-axis labels
    for (float current_point = min_left; current_point <= max_left; current_point += scale_left)
    {
        // Position the number along the left axis
        glRasterPos3f(L_AXIS, current_point*L_SCALE+X_AXIS, 0.0);
        
        // Print the number with an appropriate amount of leading space
        sprintf(left_label, "%3.0f", current_point);
        
        // Draw the number
        gl_draw(left_label, strlen(left_label));
    } // current_point
    
    // Draw the Left y-axis label
	gl_font(font_type, font_size);
    glRasterPos2f(L_AXIS-0.5, 1.5);
    //glRotatef(90.0, 0.0, 0.0, 0.0);
    gl_draw(y_left, strlen(y_left));
    
    // Draw all of the y-axis labels
    if (num_bars >= 3)
    {
    	gl_font(font_type, font_size);
        char right_label[] = "  0";
        for (float current_point = min_right; current_point <= max_right; current_point += scale_right)
        {
            // Position the number along the left axis
            glRasterPos3f(R_AXIS, current_point*R_SCALE+X_AXIS, 0.0);
            
            // Print the number with an appropriate amount of leading space
            sprintf(right_label, "%4.0f", current_point);
            
            // Draw the number
            gl_draw(right_label, strlen(right_label));
        } // current_point
        
        // Draw the Right y-axis label
	    gl_font(font_type, font_size);
        glRasterPos2f(R_AXIS-1.0, 1.5);
        //glRotatef(90.0, 0.0, 0.0, 0.0);
        gl_draw(y_right, strlen(y_right));
    }
}

