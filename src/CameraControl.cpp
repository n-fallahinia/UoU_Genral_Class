#include <iostream>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h> // For usleep
//#include <cv.h>
//#include <highgui.h>

#include "CameraControl.h"
//#include "conversions.h"
//#include "general_functions.h"

CameraControl::CameraControl()
{
        // The camera is off
        camera_status = DC1394_OFF;
        num_dma_buffers = 8;
        dc1394_handle_created = false;
        dc1394_camera_created = false;
        dc1394_capture_running = false;
        bayer_register = 0x1040;
        num_colors = 3;
}

CameraControl::~CameraControl()
{
        cleanup();
}

// Shut down the camera after use
void CameraControl::cleanup( void ) 
{
        // Stop the camera, if it is running
        if (dc1394_capture_running)
        {
                printf("Stopping the camera...\n");
                camera_error = dc1394_capture_stop(dc1394Camera);
                if (camera_error != DC1394_SUCCESS)
                {
                        printf("Could not stop camera!  Error code: %d\n", camera_error);
                        exit (-1);
                }
                dc1394_capture_running = false;
        }
        
        // Delete the camera handle, if it exists
        if (dc1394_camera_created)
        {
                printf("Freeing the camera...\n");
                dc1394_camera_free(dc1394Camera);
                dc1394_camera_created = false;
        }
        
        // Destroy the handle to the raw1394
        if (dc1394_handle_created)
        {
                printf("Freeing the dc1394 handle...");
                dc1394_free(dc1394_handle);
                printf("Done!\n");
                dc1394_handle_created = false;
        }
} // cleanup

// Start running the camera
bool CameraControl::startCamera()
{
        // Initialize loop variable
        int i;
        bool done = false;
        
        // Create new dc1394 handle
        dc1394_handle =  dc1394_new();
        dc1394_handle_created = true;
        
        /*-----------------------------------------------------------------------
        *  get the camera nodes and describe them as we find them
        *-----------------------------------------------------------------------*/
        // Get a list of all cameras available on the computer
        camera_error = dc1394_camera_enumerate(dc1394_handle, &cameras_list);
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Unable to generate a list of available cameras!\n");
                cleanup();
                return(false);
        }
        
        // Verify that at least one camera was found
        num_cameras = cameras_list->num;
        fflush(stdout);
        if (num_cameras < 1)
        {
                // if you get this message and you have multiple 1394 cards you may want to try
                // modifying the input parameter for dc1394_create_handle() above
                printf("No cameras found! (%d nodes on the bus)\n", num_cameras);
                printf("You may need to try a different 1394 device (modify code to fix)\n");
                cleanup();
                return(false);
        }
        
        // Start the first camera on the bus
        printf("Creating camera handle...");
        camera_guid = cameras_list->ids[0].guid;
        dc1394Camera = dc1394_camera_new(dc1394_handle, camera_guid);
        dc1394_camera_created = true;
        dc1394_camera_free_list(cameras_list);
        printf("Success!\n");
        
        // Turn off the auto-adjustment of the camera exposure time, shutter speed and gain factor
        //camera_error = dc1394_feature_set_mode(dc1394Camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO);
        camera_error = dc1394_feature_set_mode(dc1394Camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
        //printf("MAN::EXP: %d | ", camera_error);
        //camera_error = dc1394_feature_set_mode(dc1394Camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
        camera_error = dc1394_feature_set_mode(dc1394Camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
        //printf("SHUT: %d | ", camera_error);
        //camera_error = dc1394_feature_set_mode(dc1394Camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
        camera_error = dc1394_feature_set_mode(dc1394Camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
        //printf("GAIN: %d | ", camera_error);
        // Saturation was not affected by API v.1.3 -- should test to see if this is worth it
        //camera_error = dc1394_feature_set_mode(dc1394Camera, DC1394_FEATURE_SATURATION, DC1394_FEATURE_MODE_MANUAL);
        //printf("SAT: %d", camera_error);
        //printf("\n");
        
        // Manually set values for these parameters - The hard-coded parameters here are intended to be determined experimentally
        camera_error = dc1394_feature_set_value(dc1394Camera, DC1394_FEATURE_EXPOSURE, 330);
        //printf("SET::EXP: %d | ", camera_error);
        camera_error = dc1394_feature_set_value(dc1394Camera, DC1394_FEATURE_SHUTTER, 475);
        //printf("SHUT: %d | ", camera_error);
        camera_error = dc1394_feature_set_value(dc1394Camera, DC1394_FEATURE_GAIN, 317);
        //printf("GAIN: %d | ", camera_error);
        //camera_error = dc1394_feature_set_value(dc1394Camera, DC1394_FEATURE_SATURATION, 0);
        //printf("SAT: %d", camera_error);
        //printf("\n");
        
        // Get a list of supported  video modes for this camera
        printf("Getting a list of supported video modes for the camera...");
        camera_error = dc1394_video_get_supported_modes(dc1394Camera, &video_mode_list);
        printf("Success!\n");
        if ( camera_error !=DC1394_SUCCESS )
        {
                fprintf( stderr, "Unable to get the supported video modes\n" );
                cleanup();
                return(false);
        }
        num_video_modes = video_mode_list.num;
        printf("%d supported video modes detected.\n", num_video_modes);
        
        // Select highest-resolution mode that is MONO8
        printf("Searching for the highest resolution MONO8 mode available...");
        i = num_video_modes;
        while ((i >= 0) && (!done))
        {
                // Extract new video mode
                i--;
                current_video_mode = video_mode_list.modes[i];
                
                // Don't consider FORMAT 7 modes (i.e. "scalable")
                if ( !dc1394_is_video_mode_scalable(current_video_mode) ) 
                {
                        dc1394_get_color_coding_from_video_mode(dc1394Camera, current_video_mode, &current_color_coding);
                        if (current_color_coding == DC1394_COLOR_CODING_MONO8)
                        {
                                printf("Success!\n");
                                done = true;
                        }
                }
        }
        
        // Verify that we found a video mode that is MONO8
        dc1394_get_color_coding_from_video_mode(dc1394Camera, current_video_mode, &current_color_coding);
        if ((current_color_coding != DC1394_COLOR_CODING_MONO8))
        {
                printf("Could not get a valid MONO8 mode\n");
                cleanup();
                return(false);
        }
        printf("Current video mode is %d\n", current_video_mode);
        
#ifdef USE_1394B
        dc1394_video_set_operation_mode(dc1394Camera, DC1394_OPERATION_MODE_1394B);
        dc1394_video_set_iso_speed(dc1394Camera, DC1394_ISO_SPEED_800);
#else
        dc1394_video_set_iso_speed(dc1394Camera, DC1394_ISO_SPEED_400);
#endif
        
        // Get highest available framerate
        printf("Getting highest available framerate...");
        camera_error = dc1394_video_get_supported_framerates(dc1394Camera, current_video_mode, &framerates_list);
        if (camera_error != DC1394_SUCCESS) 
        {
                printf("Can't get a framerate\n");
                cleanup();
                return(false);
        }
        num_framerates = framerates_list.num;
        current_framerate = framerates_list.framerates[num_framerates-1];
        printf("Success!  Framerate is %d\n", current_framerate);
        
        // Set up video capture
        printf("Setting up video capture..." );
        camera_error = dc1394_video_set_mode(dc1394Camera, current_video_mode );
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Unable to set video mode = %d\n"
                        "(Check dc1394/video.h for the meaning of this value)\n",
                        current_video_mode);
                cleanup();
                return(false);
        }
        camera_error = dc1394_video_set_framerate( dc1394Camera, current_framerate );
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Unable to set framerate = %d\n"
                        "Check dc1394/types.h for the meaning of this value)\n",
                        current_framerate );
                cleanup();
                return(false);
        }
        camera_error = dc1394_capture_setup(dc1394Camera, num_dma_buffers, DC1394_CAPTURE_FLAGS_DEFAULT);
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Unable to set up video capture-\n"
                        "check line %d of %s to make sure\n"
                        "that the video mode and framerate are\n"
                        "supported by your camera\n",
                        __LINE__,__FILE__ );
                cleanup();
                return(false);
        }
        dc1394_capture_running = true;
        printf("Success!\n");
        
        // Determine the size of the images from the video mode
        camera_error = dc1394_get_image_size_from_video_mode(dc1394Camera, current_video_mode, &num_cols, &num_rows);
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Could not determine the size of the images!\n");
                cleanup();
                return(false);
        }
        printf("The camera is up.  Image size is (%dx%d) with %d colors.\n", num_cols, num_rows, num_colors);
        
        // Have the camera start sending data
        printf("Starting ISO transmission...");
        camera_error = dc1394_video_set_transmission(dc1394Camera, DC1394_ON);
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Unable to start camera iso transmission\n" );
                cleanup();
                return (false);
        }
        printf("Success!\n");
        
        // Reinitialize loop variable
        i = 0;
        
        //  Sleep untill the camera has a transmission
        printf("Waiting for camera to turn on...");
        while ((camera_status == DC1394_OFF) && (i <= 5))
        {
                // Sleep for 50 milliseconds and increment the counter
                usleep(50000);
                i++;
                
                // Check for a transmission
                camera_error = dc1394_video_get_transmission(dc1394Camera, &camera_status);
                if ( camera_error != DC1394_SUCCESS ) 
                {
                        printf("Unable to get transmision status\n" );
                        cleanup();
                        return (false);
                }
        }
        
        // If the camera still didn't turn on, quit the program
        if (camera_status == DC1394_OFF)
        {
                printf("Camera doesn't seem to want to turn on!\n");
                cleanup();
                return (false);
        }
        
        // Display the number of iterations required to start recording images
        printf("Success (%d iterations)!\n\tCapturing images\n", i);
        return (true);
} // startCamera

// Return the size of the images recorded by the camera
void CameraControl::getSize(int * n_col, int *n_row, int *n_inc)
{
        *n_col = num_cols;
        *n_row = num_rows;
        *n_inc = num_colors;
} // getSize

// Extract the last image that the camera recorded
bool CameraControl::getImage ( unsigned char * image )
{
        // Copy the image buffer to the output
        if ( image == NULL ) 
        {
                image = new unsigned char[num_colors * num_cols * num_rows];
        }
        memcpy (image, image_RGB, sizeof(unsigned char) * num_colors * num_cols * num_rows);
        
        return (true);
} // getImage

// Grabs a new image and stores it
bool CameraControl::grabImage ()
{
        static int i=0;
        i++;
        
        // Dequeue (grab) one frame
        camera_error = dc1394_capture_dequeue(dc1394Camera, DC1394_CAPTURE_POLICY_WAIT, &current_frame);
        if ( camera_error != DC1394_SUCCESS) 
        {
                printf("Unable to capture a frame\n");
                cleanup();
                return (false);
        }
        
        // Extract the frame to the image buffer
        if (!setImageBuffer())
        {
                cleanup();
                return (false);
        }
        
        // Enqueue (release) the current frame
        camera_error = dc1394_capture_enqueue(dc1394Camera, current_frame);
        if (camera_error != DC1394_SUCCESS)
        {
                fprintf( stderr, "Unable to release the current frame\n");
                cleanup();
                return (false);
        }
        
        // Successfully completed all tasks
        return (true);
} // grabImage

// Sets the 
bool CameraControl::setImageBuffer()
{
        // Allocate the input image
        image_bayer = current_frame->image;
        
        // Allocate space for the output image, if needed
        if (image_RGB == NULL )
        {
                image_RGB = new unsigned char[num_cols*num_rows*num_colors];
        }
        
        return (convert2ColorImage());
} // setImageBuffer

// Convert the Bayer image to a color (RGB) image
bool CameraControl::convert2ColorImage()
{
        // Determine the Bayer tile pattern used by the camera
        //printf("Finding the Bayer Tile Pattern...(");
        if (!getBayerTile())
        {
                printf("Could not get Bayer tile pattern from camera)\n");
                cleanup();
                return (false);
        }
        //printf(")...Success! (pattern is %d)\n", bayer_pattern);
        
        // Use Bayer pattern to convert the image to RGB8
        //printf("Converting the image to RGB8 using Bayer Decoding...");
        camera_error = dc1394_bayer_decoding_8bit(image_bayer, image_RGB, num_cols, num_rows, bayer_pattern, DC1394_BAYER_METHOD_NEAREST);
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Image not converted (%d)!\n", camera_error);
                return (false);
        }
        //printf("Success!\n");
        return (true);
} // convert2ColorImage

// Turn off the camera
bool CameraControl::turnOffCamera()
{
        // Remove images from the queue
        printf("Removing the last image from the queue...\n");
        camera_error = dc1394_capture_enqueue(dc1394Camera, current_frame);
        if (camera_error != DC1394_SUCCESS)
        {
                fprintf( stderr, "Unable to release the current frame\n");
                cleanup();
                return (true);
        }
        
        // Stop data transmission
        printf("Stopping data collection...\n");
        camera_error = dc1394_video_set_transmission(dc1394Camera, DC1394_OFF);
        if ( camera_error != DC1394_SUCCESS ) 
        {
                printf("Couldn't stop the camera?\n");
                cleanup();
                return (true);
        }
        
        // Close camera
        cleanup();
        return (false);
} // turnOffCamera

// Get the Bayer tile pattern
bool CameraControl::getBayerTile()
{
        uint32_t register_value = 0;
        
        // Query the BAYER_TILE_MAPPING register
        // For more information check the PGR IEEE-1394 Digital Camera Register Reference
        camera_error = dc1394_get_control_register(dc1394Camera, bayer_register, &register_value);
        if (camera_error != DC1394_SUCCESS)
        {
                printf("Register not read!\n");
                cleanup();
                return (false);
        }
        
        // Determine the Bayer color pattern used by the camera
        switch( register_value )
        {
                default:
                case BAYER_PATTERN_YYYY:
                        // no bayer--not MONO
                        bayer_pattern = (dc1394color_filter_t) 0;
                        //printf("Not MONO");
                        break;
                case BAYER_PATTERN_RGGB:
                        bayer_pattern = DC1394_COLOR_FILTER_RGGB;
                        //printf("RGGB");
                        break;
                case BAYER_PATTERN_GBRG:
                        bayer_pattern = DC1394_COLOR_FILTER_GBRG;
                        //printf("GBRG");
                        break;
                case BAYER_PATTERN_GRBG:
                        bayer_pattern = DC1394_COLOR_FILTER_GRBG;
                        //printf("GRBG");
                        break;
                case BAYER_PATTERN_BGGR:
                        bayer_pattern = DC1394_COLOR_FILTER_BGGR;
                        //printf("BGGR");
                        break;
        }
        return (true);
} // getBayerTile

bool 
CameraControl::setAllParameters()
{

}

bool 
CameraControl::setBrightness(double input)
{
}

bool 
CameraControl::setExposure(double input)
{
        
}

bool 
CameraControl::setGamma(double input)
{
}

bool 
CameraControl::setShutter(double input)
{
        
}

bool 
CameraControl::setFrameRate(int input)
{
        
}


bool 
CameraControl::setGain(double input)
{
        
}
        
        
bool 
setResolution(int input1, int input2)
{
        
}


bool 
CameraControl::setROI(int input)
{
        
}


bool 
CameraControl::setGrey(bool input)
{
        
}

