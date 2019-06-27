#include "Images.h"

Images::Images()
{
}

Images::~Images()
{
	if(data != NULL) 
	{
		delete [] data;
	}
	
}


Images::Images(int r, int c, int i)
{
	rows = r;
	cols = c;
	inc = i;
	
	data = new unsigned char [rows*cols*inc];
}

//Only one image
Images::Images(int r, int c, int i, unsigned char * input)
{
	rows = r;
	cols = c;
	inc = i;
	
	data = new unsigned char [rows*cols*inc];
	if ( input != NULL)
		memcpy ( data, input, sizeof(unsigned char)*rows*cols*inc );	
}

Images::Images(Images *input)
{
	rows = input->rows;
	cols = input->cols;
	inc = input->inc;
	
	data = new unsigned char [rows*cols*inc];
	input->readImage(data);
	
}


void Images::readImage(unsigned char * output)
{
	if ( output != NULL)
		memcpy ( output, data, sizeof(unsigned char)*rows*cols*inc ) ;
}

void Images::writeImage( unsigned char * input)
{
	if ( input != NULL)
		memcpy ( data, input, sizeof(unsigned char)*rows*cols*inc );

}

void Images::writeImage( int r, int c, int i, unsigned char * input)
{
	rows = r;
	cols = c;
	inc = i;
	if (data ==NULL )
		data = new unsigned char [rows*cols*inc];
	if ( input != NULL)
		memcpy ( data, input, sizeof(unsigned char)*rows*cols*inc );
		
	printf("Write an image\n");
}

