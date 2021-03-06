
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <termios.h>
#include <ncurses.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>

#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cxmisc.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv/cxmisc.h"

#include "main.h"
#include "erosion.h"
#include "dilation.h"
#include "game_behavior.h"
#include "frame_grabber.h"
#include "accelerators.h"
#include "frame_processing.h"

extern int DEBUG_MODE;
extern int DRAWING_ON;
extern int USE_V4L_CAPTURE;
extern struct capture_data capture_info;

int use_accels;
double xComLoc= 0.0;
double yComLoc= 0.0;

IplImage* rawImage = NULL;
IplImage* grayImage = NULL;
IplImage* skinImage = NULL;
IplImage* tmp_skinImage = NULL;
IplImage* yuvImage = NULL;
IplImage* ImaskCodeBook = NULL;
IplImage* ImaskCodeBookCC = NULL;
IplImage* ImaskCodeBookCCInv = NULL;
IplImage* im_gray = NULL;

//The variables below are used to map game controls
int turncount = 0;
int rightcount = 0;
int leftcount = 0;
int upcount = 0;
int downcount = 0;
int delaystart = 0;

//nomdef - Number of defects in the produced image contour
int nomdef = 0;
static const double pi = 3.14159265358979323846;
//Used as a delay variable in order to prevent constant spinning of game pieces
int fingerDelay = 0;

//variables to hold center of mass of largest 2 contours
int com_x[2];
int com_y[2];
int com_x_last[2] = {-1,-1};
int com_y_last[2] = {-1,-1};
int com_x_delta[2] = {0,0};
int com_y_delta[2] = {0,0};

//Delay calculation variables
int nloop = 1;

CvPoint2D32f frame1_features[MAX_FEATURES];
CvPoint2D32f frame2_features[MAX_FEATURES];
//Use the same count of features as used for frame 1
char optical_flow_found_feature[MAX_FEATURES];
//This is used if the resulting number of features for the second frame is not equivalent to the number of features for the first frame
float optical_flow_feature_error[MAX_FEATURES];

CvBGCodeBookModel* model = 0;
const int NCHANNELS = 3;
bool ch[NCHANNELS]={true,true,true}; // This sets what channels should be adjusted for background bounds

//This will stop the optical flow process after 20 iterations have been performed
//Decreasing this value should improve performance at the expense of accuracy
CvTermCriteria optical_flow_termination_criteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);

IplImage* contour_frame = NULL;
IplImage* frame = NULL;
IplImage* frame1 = NULL;
IplImage* frame1_1C = NULL;
IplImage* frame2_1C = NULL;
IplImage* eig_image = NULL;
IplImage* temp_image = NULL;
IplImage* pyramid1 = NULL;
IplImage* pyramid2 = NULL;
IplImage* frameX = NULL;

//These are boolean used in logic for the motion of the hand
int canTurn = 0;
bool motionLeft = false;
bool motionRight = false;
bool motionDown = false;
int motionDownCount = 0;
int motionLeftCount = 0;
int motionRightCount = 0;
int resetCount = 0;
char motionindic[] = "n";

//Variables used for codebook method of background subtraction:
//CvBGCodeBookModel* model = 0;
//const int NCHANNELS = 3;
//bool ch[NCHANNELS]={true,true,true}; // This sets what channels should be adjusted for background bounds
extern struct capture_data capture_info;
extern CvCapture* capture;

struct timeval timevalA;
struct timeval timevalB;
struct timeval timevalC;

//accelerator variables
int fd_devmem;
int fd_erode_accel;
int fd_dilate_accel;
void* frame_buffer_1;
void* frame_buffer_2;

// This function will allocate space for images.
void allocateOnDemand(IplImage **img, CvSize size, int depth, int channels){
	if(*img != NULL)
		return;

	*img = cvCreateImage(size, depth, channels);

	if(*img == NULL){
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}

// Initialize hardware accelerators specified by the use_accel parameter. This
// parameter should be set using constants defined in accelerator header file.
int init_accel(int use_accel){
	use_accels = use_accel;
	if(use_accel != USE_ACCEL_NONE){

		//open erosion accelerator device file
		if(use_accel & USE_ACCEL_ERODE){
			if(-1 == (fd_erode_accel = open(EROSION_DEV_PATH, O_RDWR | O_SYNC))){
				perror("open failed erode");
				return -1;
			}
		}

		//open dilation accelerator device file
		if(use_accel & USE_ACCEL_DILATE){
			if(-1 == (fd_dilate_accel = open(DILATION_DEV_PATH, O_RDWR | O_SYNC))){
				perror("open failed dilate");
				return -1;
			}
		}

		//open /dev/mem
		printf("Opening /dev/mem\n");
		if(-1 == (fd_devmem = open("/dev/mem", O_RDWR | O_SYNC))){
			perror("open failed");
			return -1;
		}

		//mmap frame buffers in /dev/mem
		if(-1 == (int)(frame_buffer_1 = mmap(0,FRAME_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd_devmem,FRAME_1_ADDRESS))){
			perror("mmap failed");
			return -1;
		}

		if(-1 == (int)(frame_buffer_2 = mmap(0,FRAME_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd_devmem,FRAME_2_ADDRESS))){
			perror("mmap failed");
			return -1;
		}
	}
	return 0;
}

// currently this function ensures all images are allocated and creates the initial 
// background model.
void init_frame_processing(int calib_frames){
	int nframes;

	allocateOnDemand(&rawImage, capture_info.dim, IPL_DEPTH_8U, 3);
	allocateOnDemand(&skinImage, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&grayImage, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&tmp_skinImage, capture_info.dim, IPL_DEPTH_8U, 1);

	allocateOnDemand(&contour_frame, capture_info.dim, IPL_DEPTH_8U,3);
	allocateOnDemand(&frame1_1C, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&frame1, capture_info.dim, IPL_DEPTH_8U,1);
	//allocateOnDemand(&frame2_1C, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&eig_image, capture_info.dim, IPL_DEPTH_32F, 1);
	allocateOnDemand(&temp_image, capture_info.dim, IPL_DEPTH_32F, 1);
	allocateOnDemand(&pyramid1, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&pyramid2, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&frameX, capture_info.dim, IPL_DEPTH_8U, 1);
	//allocateOnDemand(&im_gray, cvGetSize(rawImage), IPL_DEPTH_8U, 1);
	allocateOnDemand(&im_gray, capture_info.dim, IPL_DEPTH_8U, 1);

	if(DRAWING_ON){
		cvNamedWindow("Detected Skin", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Detected Skin",479,0);
		cvNamedWindow("Contours", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Contours", 640,480);
	}
}


IplImage* hsv_image = NULL;
CvScalar  hsv_min = cvScalar(0, 30, 80, 0);
CvScalar  hsv_max = cvScalar(20, 150, 255, 0);

void erode(IplImage* src, IplImage* dst){
	int temp;
	if(use_accels & USE_ACCEL_ERODE){
		//set row and column registers. this could be done in initialization.
		if(-1 == ioctl(fd_erode_accel, EROSION_SELECT_REG, EROSION_ROWS_REG)){
			perror("ioctl failed");
			return -1;
		}
		write(fd_erode_accel, &(capture_info.dim.height), 4);

		if(-1 == ioctl(fd_erode_accel, EROSION_SELECT_REG, EROSION_COLS_REG)){
			perror("ioctl failed");
			return -1;
		}
		write(fd_erode_accel, &(capture_info.dim.width), 4);

		//copy frame to be eroded into src frame buffer
		memcpy(frame_buffer_1, src->imageData, src->imageSize);

		//start the accelerator
		if(-1 == ioctl(fd_erode_accel, EROSION_ACCEL_START)){
			perror("ioctl failed");
			return -1;
		}

		//do a blocking read to wait for accelerator to finish
		read(fd_erode_accel, &temp, 4);

		//retrieve the eroded image.
		memcpy(dst->imageData, frame_buffer_2, src->imageSize);

	}else{
		cvErode(src, dst, NULL, 1);
	}
}

void dilate(IplImage* src, IplImage* dst){
	int temp;
	if(use_accels & USE_ACCEL_DILATE){
		//set row and column registers. this could be done in initialization.
		if(-1 == ioctl(fd_dilate_accel, DILATION_SELECT_REG, DILATION_ROWS_REG)){
			perror("ioctl failed");
			return -1;
		}
		write(fd_dilate_accel, &(capture_info.dim.height), 4);

		if(-1 == ioctl(fd_dilate_accel, DILATION_SELECT_REG, DILATION_COLS_REG)){
			perror("ioctl failed");
			return -1;
		}
		write(fd_dilate_accel, &(capture_info.dim.width), 4);

		//copy frame to be dilated into src frame buffer
		memcpy(frame_buffer_1, src->imageData, src->imageSize);

		//start the accelerator
		if(-1 == ioctl(fd_dilate_accel, DILATION_ACCEL_START)){
			perror("ioctl failed");
			return -1;
		}

		//do a blocking read to wait for accelerator to finish
		read(fd_dilate_accel, &temp, 4);

		//retrieve the dilated image.
		memcpy(dst->imageData, frame_buffer_2, src->imageSize);

	}else{
		cvDilate(src, dst, NULL, 1);
	}
}


char get_input(){
	int sum_x = 0;
	int sum_y = 0;
	int diff_x = 0;
	int diff_y = 0;
	int temp_x = 0;
	int temp_y = 0;
	int number_of_features;
	int i = 0;
	int temp;
	char input_char = INPUT_NONE;
	CvPoint p,q;
	bool turnaround;
	bool pieceRight;
	bool pieceLeft;
	bool pieceDown;
	turnaround = false;
	pieceRight = false;
	pieceLeft = false;
	pieceDown = false;

	//The Optical Flow window is created to visualize the output of the optical flow algorithm
	//The size of this winodw is automatically adjusted in order to match the previously determined window width and height

	if(DEBUG_MODE){
		printf("------------start of get input-------------\n");
		gettimeofday(&timevalA, NULL);
	}

	if(!USE_V4L_CAPTURE){
		rawImage = cvQueryFrame(capture);

		if(rawImage == NULL)
			return INPUT_NONE;

		cvCvtColor(rawImage, grayImage, CV_BGR2GRAY);
		allocateOnDemand(&hsv_image, capture_info.dim, IPL_DEPTH_8U, 3);
		//doing skin detection
		cvCvtColor(rawImage, hsv_image, CV_BGR2HSV);
		cvInRangeS(hsv_image, hsv_min, hsv_max, skinImage);

	}else{
#ifndef USE_MULTI_THREAD_CAPTURE
		capture_frame((void*)NULL); //if not multi-threaded must capture an image b4 query
#endif
		grayImage = v4lQueryFrame();
		cvCopy(grayImage->maskROI, skinImage);
	}

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("capture frame time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}


	//use the morphological 'open' operation to remove small foreground noise.
	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	erode(skinImage, skinImage);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("erode frame time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	dilate(skinImage, skinImage);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("dilate frame time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	//use the morphological 'close' operation to remove small background noise.
	dilate(skinImage, skinImage);
	erode(skinImage, skinImage);

	//reset the contour frame image.
	memset(contour_frame->imageData, 0, contour_frame->imageSize);
	cvCopy(skinImage, tmp_skinImage);

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	//not using HW accelerated contour area detection.
	detect(tmp_skinImage, contour_frame);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("detect() time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	//Approximate the number of fingers in the image to be equivalent to the number of defects in the contour of the frame
	int fingers = nomdef;
	if((fingers >= MIN_DEFECTS_TO_ROTATE) && canTurn){
		turncount++;
		if(turncount >= ROTATE_DETECTION_THRESHOLD){
			turnaround = true;
			turncount = 0;
			canTurn = 0;
		}
	}else if((fingers < MIN_DEFECTS_TO_ROTATE) && !canTurn){
		turncount++;
		if(turncount >= ROTATE_DETECTION_THRESHOLD){
			turnaround = false;
			turncount = 0;
			canTurn = 1;
		}
	}

	if(DRAWING_ON){
		cvShowImage("Detected Skin", skinImage);
		cvShowImage("Contours", contour_frame);
		cvWaitKey(1); //cause high gui to finish pending highgui operations. (allow images to be displayed)
	}

	int com_diff;
	//verify that the program has a valid last center of mass value, then compute COM value
	//for largest COM
	if(com_x_last[0] > 0 && com_x[0] > 0)
		com_x_delta[0] = (com_x_last[0] - com_x[0]);	
	if(com_y_last[0] > 0 && com_y[0] > 0)
		com_y_delta[0] = (com_y_last[0] - com_y[0]);

	//for next largest COM
	if(com_x_last[1] > 0 && com_x[1] > 0)
		com_x_delta[1] = abs(com_x_last[1] - com_x[1]);	
	if(com_y_last[1] > 0 && com_y[1] > 0)
		com_y_delta[1] = abs(com_y_last[1] - com_y[1]);

	if(DEBUG_MODE){
		printf("X_DELTA_COM0 = %d\n", com_x_delta[0]);
		printf("Y_DELTA_COM0 = %d\n", com_y_delta[0]);
		printf("X_DELTA_COM1 = %d\n", com_x_delta[1]);
		printf("Y_DELTA_COM1 = %d\n", com_y_delta[1]);
	}

	xComLoc = (double)com_x[0]/(double)capture_info.dim.width;
	yComLoc = (double)com_y[0]/(double)capture_info.dim.height;

	if((-com_x_delta[0]) > COM_X_DELTA_THRESHOLD){
		if(rightcount == 0){
			leftcount++;
		}else{
			leftcount = 0;
			rightcount = 0;
		}
	}else if(com_x_delta[0] > COM_X_DELTA_THRESHOLD){
		if(leftcount == 0){
			rightcount++;
		}else{
			leftcount = 0;
			rightcount = 0;
		}
	}

	if((com_y_delta[0]) > COM_Y_DELTA_THRESHOLD){
		if(downcount == 0){
			upcount++;
		}else{
			upcount = 0;
			downcount = 0;
		}
	}else if(-com_y_delta[0] > COM_Y_DELTA_THRESHOLD){
		if(upcount == 0){
			downcount++;
		}else{
			upcount = 0;
			downcount = 0;
		}
	}

	if(leftcount > COM_X_MOTION_THRESHOLD){
		pieceLeft = true;
	}else if(rightcount > COM_X_MOTION_THRESHOLD){
		pieceRight = true;
	}

	/*
	 *Disabled Y input generation.
	 *
	 * Allowing pieces to move down based on this COM calculation is not reliable!
	 * As user gestures for rotation, the Y COM varies by too much.
	 *
	if(downcount > COM_Y_MOTION_THRESHOLD)
		pieceDown = true;
	*/
	if(pieceRight){
		input_char = INPUT_RIGHT;
		pieceRight = false;
	}else if(pieceLeft){
		input_char = INPUT_LEFT;
		pieceLeft = false;
	}else if(turnaround){
		input_char = INPUT_SPIN_CW;
		turnaround = false;
	}else if(pieceDown){
		input_char = INPUT_DOWN;
		pieceDown = false;
	}else{
		input_char = INPUT_NONE;
	}

	return input_char;
}
//Function for detecting the number of contour defects in an image
int detect(IplImage* img_8uc1,IplImage* img_8uc3) {
	CvMemStorage* storage = cvCreateMemStorage();
	CvMemStorage* poly_storage[2];
	CvMemStorage* defect_storage[2];
	CvSeq* first_contour = NULL;
	CvSeq* maxitem[2] = {NULL, NULL};
	CvSeq* defects[2] = {NULL, NULL};
	double area = 0;
	double areamax[2] = {0, 0};
	int value;
	int i = 0;
	int j = 0;
	CvSeq* contour_array;
	CvPoint* ptr;

	//REMEMBER!! findContours modifies the source image when extracting contours!!
	int Nc = cvFindContours(img_8uc1,storage,&first_contour,sizeof(CvContour),CV_RETR_LIST);
	int copy_size;

	for(i = 0; i < 2; i++){
		defect_storage[i] = cvCreateMemStorage(0);
		poly_storage[i] = cvCreateMemStorage(0);
	}

	CvMemStorage* storage1 = cvCreateMemStorage(0);

	if(Nc>0){
		CvSeq* ptseq = cvCreateSeq(CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),
				sizeof(CvPoint), storage1);

		for(CvSeq* c=first_contour; c!=NULL; c=c->h_next){
			area=cvContourArea(c,CV_WHOLE_SEQ);

			//store information about largest 2 contours (face and hand?)
			//index 0 holds largest area, 1 holds next largest
			if(area > areamax[0]){
				areamax[1] = areamax[0];
				areamax[0] = area;
				maxitem[1] = maxitem[0];
				maxitem[0] = c;
			}else if(area > areamax[1]){
				areamax[1] = area;
				maxitem[1] = c;
			}
		}

		for(i = 0; i < 2; i++){
			if(areamax[i] > CONTOUR_MIN_AREA){
				maxitem[i] = cvApproxPoly(maxitem[i], sizeof(CvContour), poly_storage[i], CV_POLY_APPROX_DP, 10, 0);
				CvPoint pt0;
				CvSeq* hull;
				hull = cvConvexHull2(maxitem[i], 0, CV_CLOCKWISE, 0);
				defects[i] = cvConvexityDefects(maxitem[i], hull, defect_storage[i]);
			}
		}

		for(i = 0; i < 2; i++){
			com_x_last[i] = com_x[i];
			com_y_last[i] = com_y[i];
		}

		//attempt to calculate center of mass of points
		for(i = 0; i < 2; i++){
			if(areamax[i] > CONTOUR_MIN_AREA){
				contour_array = (CvSeq*)malloc(sizeof(CvSeq) * maxitem[i]->total);
				cvCvtSeqToArray(maxitem[i],contour_array, CV_WHOLE_SEQ);

				com_x[i] = 0;
				com_y[i] = 0;
				for(j = 0; j < maxitem[i]->total; j++){
					ptr = CV_GET_SEQ_ELEM(CvPoint, maxitem[i], j);
					com_x[i] += ptr->x;
					com_y[i] += ptr->y;
				}

				com_x[i] /= maxitem[i]->total;
				com_y[i] /= maxitem[i]->total;

				free(contour_array);
			}
		}

		if(NULL != defects[0])
			nomdef = defects[0]->total;

		if(DEBUG_MODE){
			printf("nomdef = %d\n", nomdef);
		}

		CvConvexityDefect* defectArray;  

		for(i = 0; i < 2; i++){
			for(;defects[i];defects[i] = defects[i]->h_next){  
				int temp_defects = defects[i]->total; //total number of detected defects

				if(temp_defects == 0)  
					continue;  

				// Alloc memory for array of defects    
				defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*temp_defects);  

				// Get defect set.  
				cvCvtSeqToArray(defects[i],defectArray, CV_WHOLE_SEQ);  

				// Draw marks for all the defects in the image.  
				if(DRAWING_ON){
					for(int i=0; i<temp_defects; i++){   
						cvLine(img_8uc3, *(defectArray[i].start), *(defectArray[i].depth_point),CV_RGB(0,0,164),1, CV_AA, 0);  
						cvCircle(img_8uc3, *(defectArray[i].depth_point), 5, CV_RGB(164,0,0), 2, 8,0);  
						cvCircle(img_8uc3, *(defectArray[i].start), 5, CV_RGB(164,0,0), 2, 8,0);  
						cvLine(img_8uc3, *(defectArray[i].depth_point), *(defectArray[i].end),CV_RGB(0,0,164),1, CV_AA, 0);  
					} 

					cvCircle(img_8uc3, cvPoint(com_x[0], com_y[0]), 5, CV_RGB(0,255,0), 2, 8,0);  
					cvCircle(img_8uc3, cvPoint(com_x[1], com_y[1]), 5, CV_RGB(0,255,0), 2, 8,0);  

					char txt[50];
					CvFont font;
					cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.5, 0.5, 0, 1, CV_AA);
					cvPutText(img_8uc3, "defects | area", cvPoint(0,15), &font, cvScalar(0,0,128,0));
					sprintf(txt,"%d",temp_defects);
					cvPutText(img_8uc3, txt, cvPoint(10, 30 + 15 * i), &font, cvScalar(0, 0, 128, 0)); 
					sprintf(txt,"%6.0f",areamax[i]);
					cvPutText(img_8uc3, txt, cvPoint(50, 30 + 15 * i), &font, cvScalar(0, 0, 128, 0)); 
					cvPutText(img_8uc3, "center of mass(x,y)", cvPoint(0, 60), &font, cvScalar(0, 0, 128, 0)); 
					sprintf(txt,"%d,%d",com_x[i],com_y[i]);
					cvPutText(img_8uc3, txt, cvPoint(0, 75 + 15 * i), &font, cvScalar(0, 0, 128, 0)); 
				}

				// Free memory.         
				free(defectArray);  
			} 
		}

		cvReleaseMemStorage(&storage1);
		for(i = 0; i < 2; i++){
			cvReleaseMemStorage(&defect_storage[i]);
			cvReleaseMemStorage(&poly_storage[i]);
		}
	}

	cvReleaseMemStorage(&storage);
	return 0;
}

