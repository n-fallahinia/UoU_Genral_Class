#include "ImageDisplay.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include "cv.h"

ImageDisplay::ImageDisplay(int x,int y,int w,int h,const char *l)
				: Fl_Box( x, y, w, h, l ) 
            //: Fl_Gl_Window(x,y,w,h,l)
{
	display_x = x;
	display_y = y;
	display_w = w;
	display_h = h;
	inc = 3;
	flag_display = 0;
	if (image == NULL)
	{
	        image = new unsigned char [w*h*inc];
        }
	
	counter = 0;
	printf("Image Display class is up\n");
}

ImageDisplay::~ImageDisplay (  )
{
	if ( image !=NULL )
	{
		delete []image;
	} 
	
}

void ImageDisplay::readImage ( unsigned char *read_image, int w, int h, int d )
{
	if ( w == display_w && h == display_h && d == inc )
	{
		memcpy ( image, read_image, sizeof(unsigned char)*w*h*d);
		//for (int i=0; i<w*h*d; i++)
			//image[i] = 255; 
	}
	else
	{
		//Resize the input image to a display image
		resizeImage( read_image, w, h, d );
	}
	flag_display = 1;
}

// Resize the image
void ImageDisplay::resizeImage ( unsigned char *read_image, int w, int h, int d )
{
	//IplImage *imgI = cvCreateImage(cvSize(w, h),IPL_DEPTH_8U,3);
//	IplImage *imgI = cvCreateImageHeader(cvSize(w, h),IPL_DEPTH_8U,3);
//	IplImage *imgJ = cvCreateImageHeader(cvSize(display_w,display_h),IPL_DEPTH_8U,3);
	//IplImage *imgJ = cvCreateImage(cvSize(display_w,display_h),IPL_DEPTH_8U,3);

//	cvSetData( imgI , read_image , w*3*sizeof(unsigned char)); 
//	cvSetData( imgJ , image , display_w*3*sizeof(unsigned char)); 

	//Resize the imgI to imgJ to have 640x480
//	cvResize( imgI, imgJ, CV_INTER_CUBIC );
	//memcpy(image, temp_image, sizeof(unsigned char)*display_w*display_h*inc);
	
	//cvReleaseImage(&imgI);
	//cvReleaseImage(&imgJ);
//	cvReleaseImageHeader(&imgI);
//	cvReleaseImageHeader(&imgJ);
} // resizeImage

// Draw the image in the window
void ImageDisplay::draw()
{
	//fl_draw_image((const uchar*)&pixbuf, 0, 0, XSIZE, YSIZE, 3, XSIZE*3);	
	
	//fl_draw_image ( image, display_x, display_y, display_w, display_h, inc, display_w*3 );
	//image = new unsigned char [512*384*3];
	/*
	if ( counter == 30 )
	{
		ppm_write("a.ppm", display_w, display_h, image);
		counter =0;
	}
	else
	{
		counter++;
	}
	*/
	if ( flag_display )
	{
	//	printf("Redraw\n");
		fl_draw_image ( image, display_x, display_y, display_w, display_h, 3, display_w*3);
	}
} // draw()

