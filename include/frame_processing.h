#ifndef __H_FRAME_PROCESSING_H__
#define __H_FRAME_PROCESSING_H__

#define CONTOUR_MIN_AREA			5000 //minimum area a contour must have to be used for gesture detection
#define MIN_DEFECTS_TO_ROTATE		4	//number of defects in a frame to detect
#define ROTATE_DETECTION_THRESHOLD	4	//number of positive detections required to recognize input as rotation
#define MAX_FEATURES				100
#define X_SUM_THRESHOLD				25
#define Y_SUM_THRESHOLD				25
#define X_MOTION_THRESHOLD			2	//min number of frames that need to detect same x-direction motion b4 output is given
#define X_DELTA_MIN					0//4
#define X_DELTA_MAX					10
#define Y_DELTA_MIN					0//4
#define Y_DELTA_MAX					10

int init_accel(int use_accel);
void init_frame_processing(int calib_frames);
void allocateOnDemand(IplImage **img, CvSize size, int depth, int channels);
int detect(IplImage* img_8uc1,IplImage* img_8uc3, int use_accel);
char get_input();

#endif
