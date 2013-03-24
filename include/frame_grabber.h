#include "opencv/cv.h"

#ifndef __H_FRAME_GRABBER_H__
#define __H_FRAME_GRABBER_H__

#define IMAGEWIDTH	640
#define IMAGEHEIGHT	480
#define BUFF_COUNT	3
#define FILE_VIDEO 	"/dev/video0"

#define CAPTURE_NAME_480	"480"
#define CAPTURE_NAME_720	"720"
#define CAPTURE_NAME_1080	"1080"

#define CAPTURE_WIDTH_480	640
#define CAPTURE_WIDTH_720	1280
#define CAPTURE_WIDTH_1080	1920

#define CAPTURE_HEIGHT_480	480
#define CAPTURE_HEIGHT_720	720
#define CAPTURE_HEIGHT_1080	1080

#define CAPTURE_SIZE_480	CAPTURE_WIDTH_480 * CAPTURE_HEIGHT_480 * 3
#define CAPTURE_SIZE_720	CAPTURE_WIDTH_720 * CAPTURE_HEIGHT_720 * 3
#define CAPTURE_SIZE_1080	CAPTURE_WIDTH_1080 * CAPTURE_HEIGHT_1080 * 3

#define MAX_NAME_LEN	50

struct capture_data{
	char name[MAX_NAME_LEN];
	int size;	//number of bytes in captured frame
	CvSize dim;	//store width and height of frame
	//int width;	//width in pixels
	//int height;	//height in pixels
};

struct buffer{
	void * start;
	unsigned int length;
};

int init_v4l2();
int init_capture_data(char* name);
void* capture_frame(void* param);
IplImage* v4lQueryFrame();

#endif
