
#include "TimePlot.h"
#include "Constants.h"

#include <FL/Fl.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include <GL/glu.h>
#include <FL/glut.H>

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <stdio.h>

TimePlot::TimePlot(int x,int y,int w,int h,const char *l)
            : Fl_Gl_Window(x,y,w,h,l)
{
    // Define the scale parameters
    width = 20.0;
    height = 20.0;
    
    // Initialize the actual number of data points in the queue
    data_size = 0;
    desired_size = 0;
    
    // Assign various strings
    sprintf(instruction_string, "%s", "Press \"Start Controller\" to begin");
    sprintf(title, "%s", "x-Coordinate of Position over Time");
    sprintf(xlabel, "%s", "Time (seconds)");
    sprintf(ylabel, "%s", "Position (mm)");
    
    // Assign colors for plotting
    desired_color[0] = 0.25;
    desired_color[1] = 0.25;
    desired_color[2] = 0.25;
    data_color[0] = 0.0;
    data_color[1] = 1.0;
    data_color[2] = 0.0;
    line_color[0] = 1.0;
    line_color[1] = 1.0;
    line_color[2] = 1.0;
    
    // Set scale and offset values for plotting
    x_scale = 0.3;
    y_scale = 1000.0;
    x_offset = 9.0;
    y_offset = 0.0;
}

// Read a new set of data values
void TimePlot::readData(float* current_position)
{
    // Convert the current_position array to a ReadingDisplay structure
	ReadingDisplay current_data;
	current_data.data = current_position[0];
	current_data.data_x = current_position[1];
	
	// If needed, remove the first display point
	while (data_display.size() >= NUM_DATA)
	{
		data_display.pop();
		data_size--;
	}
	
	// Add the new point to the display
	data_display.push(current_data);
	data_size++;
	//std::cout << " [" << current_position[1] << "," << current_position[0] << "]\n";
} // readData

// Clear the current data queue
void TimePlot::clear_data()
{
    while (!data_display.empty())
    {
        data_display.pop();
        data_size--;
    }
} // clear_data

// Clear the current desired data vector
void TimePlot::clear_desired()
{
    desired_data.clear();
    desired_size = 0;
} // clear_data

// Read a new set of desired_data values
void TimePlot::readDesiredData(float* desired_position)
{
    // Convert the desired_position array to a ReadingDisplay structure
	ReadingDisplay current_data;
	current_data.data = desired_position[0];
	current_data.data_x = desired_position[1];
	//std::cout << "\t" << desired_position[1] << "\t" << desired_position[0] << "\t" << current_data.data_x << "\t" << current_data.data << "\n";
	
	// Add the new point to the display
	desired_data.push_back(current_data);
	desired_size++;
} // readDesiredData

// Render the data according to options
void TimePlot::draw() 
{
    //std::cout << "Re-drawing TimePlot\n";
    // If the window is not valid, re-initialize it
    if (!valid())
    {
        glLoadIdentity();
        gluOrtho2D(-width/2.0, width/2.0, -height/2.0, height/2.0);
    }
    
    // Set the color
    glColor3f(0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    // Draw the axes
	//drawZeroLines();
	drawYLabel();
	
	// Draw the desired data
	drawDesired(0, 0);
	
	// Draw the data
	drawData(0, 0);
	
	// Draw the instructions if no trajectory data has been input
    if ((desired_size == 0) && (data_size == 0))
    {
		drawInstruction();
    }
} // draw

//Draw data as dots
void TimePlot::drawData(int force_index, int figure_index)
{
	glBegin(GL_LINE_STRIP);
    
 	for (int dataIdx=0; dataIdx<data_size; dataIdx++)
	{
	    // If no data exists, 
		if (data_display.empty())
		    return;
	    
		ReadingDisplay curr;
		curr = data_display.front();
		data_display.pop();
		data_display.push(curr);
		//float x = curr.data_x;
		//x = x/1000.0;
		float x = curr.data_x;
		float y = curr.data;
		glColor3f(data_color[0],data_color[1],data_color[2]);
		//glVertex3f( x-10, y*scale_y + start_y[figure_index], 0.0 );
		glVertex2f(x_scale*x-x_offset, y_scale*y-y_offset);
		//cout << "X = " << x << "Y= " << y << "\n";
	} // dataIdx
    glEnd();
    
//    glutSolidSphere(10,20,20);
    
}//drawDots

// Draw desired data as lines
void TimePlot::drawDesired(int force_index, int figure_index)
{
	glBegin(GL_LINE_STRIP);
    //std::cout << "Drawing desired data!\n";
 	for (int dataIdx = 0; dataIdx < desired_size; dataIdx++)
	{
	    // If no data exists, 
		if (desired_data.empty())
		    return;
	    
	    // Extract the current data value
		ReadingDisplay curr;
		curr = desired_data[dataIdx];
		
		// Read the (x,y) coordinates
		float x = curr.data_x;
		float y = curr.data;
		
		// Draw the point
		glColor3f(desired_color[0],desired_color[1],desired_color[2]);
		//glVertex3f( x-10, y*scale_y + start_y[figure_index], 0.0 );
		glVertex2f(x_scale*x-x_offset, y_scale*y-y_offset);
		//cout << "X = " << x << "Y= " << y << "\n";
	} // dataIdx
    glEnd();
    
//    glutSolidSphere(10,20,20);
    
}//drawDots

void 
TimePlot::drawTitle()
{
	//Title
	gl_font(1, 12);
	glColor3f(line_color[0],line_color[1],line_color[2]);
	//glRasterPos3f(-4.0, height - 0.5, 0); 
	glRasterPos2f(-4.0, height - 0.5);
	gl_draw(title, strlen(title));
}//drawTitle


void 
TimePlot::drawXLabel()
{
	/*
    int tickers = 10;
    int t_total = data_x[data_num -1] - data_x[0];
    float interval = floor( float(t_total)/float(tickers) );
    gl_font(1, 12);
    glColor3f(1.0, 1.0, 1.0);
    char p[5];
    for ( int i=0; i< tickers-2; i++)
    {   
        glRasterPos3f(float(i)*width/float(tickers) - width/2.0 , 0.0, 0);
        sprintf( p, "%d", i*interval );
        gl_draw(p, strlen(p));
    }
    glRasterPos3f(width/2.0 - 1, 0.0, 0);
    sprintf( p, "t (s)");
    gl_draw(p, strlen(p));
    */
    //May want to draw a line to show the x axis
}//drawXLabel

// Draws the y-label on the plot
void TimePlot::drawYLabel()
{
    float x = -width/2.0 + 0.5;
    float y;
    
    //Label the zero line
    gl_font(1, 12);
    glColor3f(line_color[0],line_color[1],line_color[2]);
    char p[10];
	for ( int i=-9; i<10; i= i+3 )
    {
        //glRasterPos3f(x, ((float) i)*scale_y + start_y[k], 0.0);
        y = (float) i;
        glRasterPos2f(x, y);
	 	sprintf(p, "%d", i);
	  	gl_draw(p, strlen(p));
    }
    
    // Draw the y-axis label
    //glRasterPos2f(-x_offset, -y_offset);
    // Rotate?
    //gl_draw(ylabel, strlen(ylabel));
} // drawYLabel

// Draws the x-axis on the plot
void TimePlot::drawZeroLines(  )
{
    // Set up the x-axis coordinates
	float zero0[2], zero1[2];
    zero0[0] = -width/2.0;
    zero0[1] = 0.0;
    zero1[0] = width/2.0;
    zero1[1] = 0.0;
    
    glColor3f(line_color[0],line_color[1],line_color[2]);
    glBegin(GL_LINES);
      glVertex2fv(zero0);
      glVertex2fv(zero1);
    glEnd();
} // drawZeroLines

// Draws the phrase used before the experiment begins
void TimePlot::drawInstruction()
{
    // Change font and color
    gl_font(3,12);
    glColor3f(line_color[0],line_color[1],line_color[2]);
    
    // Move to the left
    glRasterPos2f(-5, 0);
    gl_draw(instruction_string, strlen(instruction_string));
}//drawInstruction


