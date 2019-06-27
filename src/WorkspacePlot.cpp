// Class function definitions for WorkspacePlot
//  Used to plot Maglev position

#include "WorkspacePlot.h"
#include "Constants.h"

#include <FL/Fl.H>
#include <FL/gl.h>
#include <FL/glut.H>
#include <FL/fl_draw.H>
#include <GL/glu.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

// Standard Constructor
WorkspacePlot::WorkspacePlot(int x,int y,int w,int h,const char *l) : Fl_Gl_Window(x,y,w,h,l)
{
    // Set clicking variables -- these may no longer be needed, eventually
    click_x = 0.0;
    click_y = 0.0;
    vx = 0.0;
    vy = 0.0;
    vz = 0.0;
    
    // Dimensions of the window
    window_width = 2.0*MAX_RADIUS + 5.0;
    axis_end = window_width*5.0/11.0;
    
    // Font definition
    font = GLUT_STROKE_ROMAN;
    
    // Define the base pose
    base_pose[0] = 0.0;
    base_pose[1] = 0.0;
    base_pose[2] = 0.0;
    
    // Define the color for the base axes (white)
    base_color[0] = 1.0;
    base_color[1] = 1.0;
    base_color[2] = 1.0;
    base_size = 3.0;
    
    // Define the default Maglev pose
    axis_pose[0] = 0.0;
    axis_pose[1] = 0.0;
    axis_pose[2] = 0.0;
    
    // Define the color for the axes that represent the MLHD
    pos_axis_color[0] = 1.0;
    pos_axis_color[1] = 0.0;
    pos_axis_color[2] = 0.0;
    neg_axis_color[0] = 1.0;
    neg_axis_color[1] = 0.0;
    neg_axis_color[2] = 1.0;
    zer_axis_color[0] = 0.0;
    zer_axis_color[1] = 1.0;
    zer_axis_color[2] = 0.0;
    axis_size = 2.0;
    
    // Set the scale values
    scale_position = 1000.0;
} // WorkspacePlot Constructor

// Draw the workspace
void WorkspacePlot::draw()
{
    // Set up the viewing frame
    if (!valid())
    {
        // Set up a 2D frame
        glLoadIdentity();
        gluOrtho2D(-window_width/2.0, window_width/2.0, -window_width/2.0, window_width/2.0);
    }
    
    // Clear the color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Draw the axes
    glPushMatrix();
        // Draw the base axes in the appropriate color
        drawAxes(base_pose, base_color, base_size, true);
        
        // Draw the current position axes in the appropriate color
        if (axis_pose[2] > AXIS_TOL/2.0)
        {
            drawAxes(axis_pose, pos_axis_color, axis_size, false);
        }
        else if (axis_pose[2] < -AXIS_TOL)
        {
            drawAxes(axis_pose, neg_axis_color, axis_size, false);
        }
        else
        {
            drawAxes(axis_pose, zer_axis_color, axis_size, false);
        }
    glPopMatrix();
}//draw

// Set the axis pose
void WorkspacePlot::set3DAxis(double pose[3])
{
    for (int dirIdx = 0; dirIdx < 3; dirIdx++)
    {
        axis_pose[dirIdx] = ((float)pose[dirIdx]) * scale_position;
    }
} // get3DAxis
void WorkspacePlot::set3DAxis(float pose[3])
{
    for (int dirIdx = 0; dirIdx < 3; dirIdx++)
    {
        axis_pose[dirIdx] = pose[dirIdx] * scale_position;
    }
} // get3DAxis
void WorkspacePlot::set3DAxis(std::vector<double> pose)
{
    if (pose.size() != 3)
    {
        std::cout << "Invalid input vector!\n";
        exit(1);
    }
    for (int dirIdx = 0; dirIdx < 3; dirIdx++)
    {
        axis_pose[dirIdx] = ((float)pose[dirIdx]) * scale_position;
    }
} // get3DAxis
void WorkspacePlot::set3DAxis(std::vector<float> pose)
{
    if (pose.size() != 3)
    {
        std::cout << "Invalid input vector!\n";
        exit(1);
    }
    for (int dirIdx = 0; dirIdx < 3; dirIdx++)
    {
        axis_pose[dirIdx] = pose[dirIdx] * scale_position;
    }
} // get3DAxis

// Set a single axis value
void WorkspacePlot::set_axis(double pose, int axisIdx)
{
    if ((axisIdx >= 0) && (axisIdx < 3))
    {
        axis_pose[axisIdx] = ((float)pose) * scale_position;
    }
} // get3DAxis
void WorkspacePlot::set_axis(float pose, int axisIdx)
{
    if ((axisIdx >= 0) && (axisIdx < 3))
    {
        axis_pose[axisIdx] = pose * scale_position;
    }
} // get3DAxis

// Draw a single axis
void WorkspacePlot::drawAxes(float pose[3], float color[3], float size, bool base_frame)
{
    // Set the color to the input
    glColor3f(color[0], color[1], color[2]);
    
    // Draw the current axes
    glPushMatrix();
        // Set the line width to the appropriate size
        glLineWidth(size);
        
        // Translate the new axes to the correct location
        glTranslatef(pose[0], pose[1], 0.0);
        
        // Draw the x-axis
        glBegin(GL_LINES);
            glVertex2f(0.0, 0.0);
            glVertex2f(axis_end, 0.0);
        glEnd();
        
        // Draw the y-axis
        glBegin(GL_LINES);
            glVertex2f(0.0, 0.0);
            glVertex2f(0.0, axis_end);
        glEnd();
        
    // Replace the original parameters
    glPopMatrix();
    
    // Draw the circle that represents the Maglev boundary at this height
    if (fabs(pose[2]) < MAX_RADIUS)
    {
        // Calculate the radius of the circle
        float radius = sqrt(MAX_RADIUS*MAX_RADIUS - pose[2]*pose[2]);
        
        // Move to a new frame
        glPushMatrix();
            // Set the line width to the appropriate size
            glLineWidth(size);
            
            // Draw the lines for the circle
            glBegin(GL_LINES);
                float theta = 0.0;
                float delta_theta = 2.0*PI / ((double)NUM_LINES);
                glVertex2f(radius*cos(theta), radius*sin(theta));
                while (theta < 2.0*PI)
                {
                    glVertex2f(radius*cos(theta+delta_theta), radius*sin(theta+delta_theta));
                    theta += delta_theta;
                    glVertex2f(radius*cos(theta), radius*sin(theta));
                }
                glVertex2f(radius*cos(theta+delta_theta), radius*sin(theta+delta_theta));
            glEnd();
        // Move back to the base frame
        glPopMatrix();
    }
    
    // For the base frame, draw the text labels of the axes
    if (base_frame)
    {
        // Set the line width to the appropriate size
        glLineWidth(2.0);
        
        // Draw the text "X"
        glPushMatrix();
            glTranslatef(axis_end, 0.0, 0.0);
            glScalef(0.012, 0.012, 1.0);
            glutStrokeCharacter(font, 'X');
        glPopMatrix();        
        
        // Draw the text "Y"
        glPushMatrix();
            glTranslatef(0.0, axis_end, 0.0);
            glScalef(0.012, 0.012, 1.0);
            glutStrokeCharacter(font, 'Y');
        glPopMatrix();
    }
} // draw3DAxis

