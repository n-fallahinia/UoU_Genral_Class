#ifndef CAMERACONTROL_H
#define CAMERACONTROL_H

#include <dc1394/control.h>
//#include "camera_parameters.h"

// define USE_1394B if you are using a 1394B camera
//#define USE_1394B

#define BAYER_PATTERN_YYYY 0x59595959 // FLEA2 should be this one
#define BAYER_PATTERN_RGGB 0x52474742 // FLEA should be this one
#define BAYER_PATTERN_GBRG 0x47425247
#define BAYER_PATTERN_GRBG 0x47524247
#define BAYER_PATTERN_BGGR 0x42474752

#define CAMERA_FREQUENCY 10.0

class CameraControl
{
    public:
        /////////////////////////////////////////////////
        // Variables
        
        unsigned int                    num_cols;
        unsigned int                    num_rows;
        int                             num_colors;
        
        /////////////////////////////////////////////////
        // Functions
        
        CameraControl ();
        //CameraControl ( int, int );
        void getSize(int *, int *, int *);
        
        ~CameraControl ();
        bool initCamera ();
        bool startCamera ();
        //bool set
        bool grabImage ();		// Take images to image_buffer
        bool getImage ( unsigned char * );		// Get the image buffer to user
        bool turnOffCamera ();
        
        bool setAllParameters();
        bool setBrightness(double);
        bool setExposure(double);
        bool setGamma(double);
        bool setShutter(double);
        bool setFrameRate(int);
        bool setGain(double);
        
        bool setResolution(int, int);
        bool setROI(int);
        bool setGrey(bool);
        
    private:
        /////////////////////////////////////////////////
        // Variables
        
        dc1394_t *                      dc1394_handle;
        dc1394camera_t *	        dc1394Camera;
        dc1394error_t                   camera_error;
        dc1394camera_list_t *           cameras_list;
        uint64_t                        camera_guid;
        uint32_t                        num_cameras;
        uint32_t                        num_video_modes;
        uint32_t                        num_framerates;
        uint32_t                        num_dma_buffers;
        uint64_t                        bayer_register;
        dc1394video_frame_t *           current_frame;
        dc1394video_frame_t *           converted_frame;
        dc1394video_modes_t             video_mode_list;
        dc1394video_mode_t              current_video_mode;
        dc1394color_coding_t            current_color_coding;
        dc1394framerates_t              framerates_list;
        dc1394framerate_t               current_framerate;
        dc1394switch_t                  camera_status;
        dc1394color_filter_t            bayer_pattern;
        bool				dc1394_camera_created;
        bool                            dc1394_handle_created;
        bool				dc1394_capture_running;
        char                            device[50];
        //   	int				numCameras;
        
        int				mode;
        int				bytes_per_pixel;
        int				format;
        int				frame_rate;
        bool				camera_on;
        unsigned char *			image_bayer;
        unsigned char *			image_RGB;
        
        /////////////////////////////////////////////////
        // Functions
        
        bool setImageBuffer ();
        bool convert2ColorImage ();
        void cleanup ();
        bool getBayerTile();
};

#endif

