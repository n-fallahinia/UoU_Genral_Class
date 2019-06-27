#ifndef IMAGES_H
#define IMAGES_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


class Images
{
public:
	int cols;
	int rows;
	int inc;
//	int num;
	
//	int max_num;
	
	
	Images();
	Images(int, int, int );
	Images(int, int, int, unsigned char *);
	Images(Images *);
	~Images();
	
	void readImage(unsigned char *);
	void writeImage(int, int, int, unsigned char *);
	void writeImage(unsigned char *);
	
	
private:
	unsigned char *data;
	
};

// Write the data array as a PPM image file
inline bool ppm_write(char *filename, int width, int height, unsigned char* data)
{
        // Declare the variables
	int num;
	int size = width * height * 3;
        
        // Attempt to open the file for writing
	FILE *fp = fopen(filename, "wb");
	if (!fp) 
	{
		printf("Cannot open file %s for writing\n",filename);
		exit(0);
	}
        
        // Write the first lines that declare the file to be a PPM
	fprintf(fp, "P6\n%d %d\n%d\n", width, height, 255);
        
        // Write the entire array
	num = (int) fwrite((void *) data, 1,  size, fp);
	
	// Close the file
	fclose(fp);
        
        // Determine if the fwrite command wrote the correct number of points
	if (num == size) 
	{
	        return(true);
        }
        else
	{
		return(false);
	}
} // ppm_write

#endif
