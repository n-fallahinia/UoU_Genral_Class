#ifndef IMAGEDISPLAY_H
#define IMAGEDISPLAY_H

#include <stdio.h>
#include <FL/Fl.H>
//#  include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Box.H>
#include <stdlib.h>
#include <FL/fl_draw.H>

class ImageDisplay : public Fl_Box	//Fl_Gl_Window 
{
public:
	int display_x, display_y; // top-left corner 
	int display_w, display_h;
	int inc;
	int flag_display;
	
//	ImageDisplay( );
	ImageDisplay(int x,int y,int w,int h,const char *l=0);
	
//	ImageDisplay(int, int, int, int, int);//x, y, w, h, and inc for display
//	ImageDisplay(unsigned char *, int, int, int, int, int);//x, y, w, h, and inc for display
	~ImageDisplay();
	void draw();   
	void readImage (  unsigned char *read_image, int w, int h, int d );
	
private:
	unsigned char *image;
	int counter;
	void resizeImage ( unsigned char *read_image, int w, int h, int d );
	
};

#endif
