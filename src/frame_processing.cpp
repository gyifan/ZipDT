
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
#include "contour_area.h"
#include "erosion.h"
#include "game_behavior.h"
#include "frame_grabber.h"
#include "accelerators.h"
#include "frame_processing.h"

extern int DEBUG_MODE;
extern int DRAWING_ON;
extern int USE_V4L_CAPTURE;

int use_accels;

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
int fd_area_accel;
int fd_erode_accel;
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

		//open contour area accelerator device file
		if(use_accel & USE_ACCEL_AREA){
			if(-1 == (fd_area_accel = open(AREA_DEV_PATH, O_RDWR | O_SYNC))){
				perror("open failed");
				return -1;
			}
		}

		//open erosion accelerator device file
		if(use_accel & USE_ACCEL_ERODE){
			if(-1 == (fd_erode_accel = open(ERODE_DEV_PATH, O_RDWR | O_SYNC))){
				perror("open failed");
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

	/*
	//create background model
	cvSet(ImaskCodeBook,cvScalar(255));

	model = cvCreateBGCodeBookModel();
	//Set color thresholds to default values
	model->modMin[0] = 3;
	model->modMin[1] = model->modMin[2] = 3;
	model->modMax[0] = 10;
	model->modMax[1] = model->modMax[2] = 10;
	model->cbBounds[0] = model->cbBounds[1] = model->cbBounds[2] = 10;

	for(nframes = 0; nframes <= nframesToLearnBG; nframes++){
	if(!DEBUG_MODE){
	mvprintw(0,0, "Creating background model. Move a little to create a more stable BG model.");
	mvprintw(1,0, "%d frames to go.", nframesToLearnBG - nframes);
	refresh();
	}else{
	printf("generating BG model, %d frames to go.\n", nframesToLearnBG - nframes);
	}

	if(!USE_V4L_CAPTURE){
	rawImage = cvQueryFrame(capture);
	}else{
#ifndef USE_MULTI_THREAD_CAPTURE
capture_frame((void*)NULL); //if not multi-threaded must capture an image b4 query
#endif
rawImage = v4lQueryFrame();
}

if(!USE_V4L_CAPTURE)
cvCvtColor(rawImage, yuvImage, CV_BGR2YCrCb);//YUV For codebook method
else
cvCvtColor(rawImage, yuvImage, CV_RGB2YCrCb);//YUV For codebook method. V4L query returns RGB format

if(nframes < nframesToLearnBG){
cvBGCodeBookUpdate(model, yuvImage);
}else if(nframes == nframesToLearnBG)
cvBGCodeBookClearStale(model, model->t/2);
}
cvCopy(ImaskCodeBook,ImaskCodeBookCC);	
*/
	if(DRAWING_ON){
		cvNamedWindow("Detected Skin", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Detected Skin",479,0);
		cvNamedWindow("Contours", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Contours", 640,480);
	}
}

//dirty function for debugging hardware accelerators. REMOVE THIS IN FINAL VERSION
void store_image_data_to_file(IplImage *img, const char *filename){
	int output_fd;
	output_fd = open(filename, O_CREAT | O_TRUNC | O_WRONLY);

	printf("store_image_data_to_file wrote %d bytes to %s\n", write(output_fd, img->imageData, img->imageSize), filename);

	close(output_fd);
}


IplImage* hsv_image = NULL;
CvScalar  hsv_min = cvScalar(0, 30, 80, 0);
CvScalar  hsv_max = cvScalar(20, 150, 255, 0);

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


//	store_image_data_to_file(skinImage, "./skin_before_erode.dat");

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	//use the morphological 'open' operation to remove small foreground noise.
	if(use_accels & USE_ACCEL_ERODE){
		//set row and column registers. this could be done in initialization.
		if(-1 == ioctl(fd_erode_accel, SELECT_REG, ROWS_REG)){
			perror("ioctl failed");
			return -1;
		}
		write(fd_erode_accel, &(capture_info.dim.height), 4);

		if(-1 == ioctl(fd_erode_accel, SELECT_REG, COLS_REG)){
			perror("ioctl failed");
			return -1;
		}
		write(fd_erode_accel, &(capture_info.dim.width), 4);

		//copy frame to be eroded into src frame buffer
		memcpy(frame_buffer_1, skinImage->imageData, skinImage->imageSize);
		
		//start the accelerator
		if(-1 == ioctl(fd_erode_accel, ACCEL_START)){
			perror("ioctl failed");
			return -1;
		}

		//do a blocking read to wait for accelerator to finish
		read(fd_erode_accel, &temp, 4);
	
		//retrieve the eroded image.
		memcpy(skinImage->imageData, frame_buffer_2, skinImage->imageSize);

	}else{
		cvErode(skinImage, skinImage, NULL, 1);
	}

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("erode frame time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

//	store_image_data_to_file(skinImage, "./skin_after_erode.dat");

//	exit(-1);


	cvDilate(skinImage, skinImage, NULL, 1);

	//use the morphological 'close' operation to remove small background noise.
	//cvDilate(skinImage, skinImage, NULL, 1);
	//cvErode(skinImage, skinImage, NULL, 1);


	//reset the contour frame image.
	memset(contour_frame->imageData, 0, contour_frame->imageSize);
	cvCopy(skinImage, tmp_skinImage);

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	if(use_accels & USE_ACCEL_AREA){
		if(detect(tmp_skinImage, contour_frame, 1)){
			printf("detect failed\n");
		}		
	}else{
		if(detect(tmp_skinImage, contour_frame, 0)){
			printf("detect failed\n");
		}
	}

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

	//THIS ISN'T GREAT--
	//Not currently taking into account the number of COM that are currently detected.
	//THIS MUST BE FIXED

	int com_diff;
	//verify that the program has a valid last center of mass value
	if(com_x_last[0] > 0 && com_y_last[0] > 0
			&& com_x_last[1] > 0 && com_y_last[1] > 0){
		for(i=0; i < 2; i++){
			com_diff = com_x_last[i] - com_x[i];
			if(abs(com_diff) > COM_X_DIFF_THRESHOLD){
				com_x_delta[i] = com_diff;
			}else{
				com_x_delta[i] = 0;
			}

		}
	}

	if(DEBUG_MODE){
		printf("X_DELTA_COM0 = %d\n", com_x_delta[0]);
		printf("Y_DELTA_COM0 = %d\n", com_y_delta[0]);
		printf("X_DELTA_COM1 = %d\n", com_x_delta[1]);
		printf("Y_DELTA_COM1 = %d\n", com_y_delta[1]);
	}


/*
	//control logic starts here			
	if(abs(sum_x) > X_SUM_THRESHOLD){
		if(sum_x < 0){
			if(rightcount == 0){
				leftcount++;
			}else{
				leftcount = 0;
				rightcount = 0;
			}
		}else{
			if(leftcount == 0){
				rightcount++;
			}else{
				leftcount = 0;
				rightcount = 0;
			}
		}
	}

	if(leftcount > X_MOTION_THRESHOLD){
		pieceLeft = true;
		leftcount = 0;
	}else if(rightcount > X_MOTION_THRESHOLD){
		pieceRight = true;
		rightcount = 0;
	}

	//if(abs(sum_y) > Y_SUM_THRESHOLD){
	//	if(sum_y < 0){
	//		pieceDown = true;
	//	}else{
	//
	//	}
	//}
*/

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

	if(leftcount > COM_X_MOTION_THRESHOLD){
		pieceLeft = true;
	}else if(rightcount > COM_X_MOTION_THRESHOLD){
		pieceRight = true;
	}

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
int detect(IplImage* img_8uc1,IplImage* img_8uc3, int use_accel) {
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
			if(use_accel){
				//select the total elements register register
				//This must be loaded with the number of elements in the contour.
				if(-1 == ioctl(fd_area_accel, SELECT_REG, ELEM_COUNT_REG)){
					perror("ioctl failed");
					return -1;
				}

				//write number of elements to register
				write(fd_area_accel, &(c->total), 4);
				copy_size = c->elem_size * c->first->count;

				//copy point sequence into first buffer
				memcpy(frame_buffer_1, c->first->data, copy_size);

				//start the accelerator
				if(-1 == ioctl(fd_area_accel, ACCEL_START)){
					perror("ioctl failed");
					return -1;
				}

				//select the upper word of the return register
				if(-1 == ioctl(fd_area_accel, SELECT_REG, RETURN_REG)){
					perror("ioctl failed");
					return -1;
				}
				read(fd_area_accel, &value, 4);
				area = (double)(-value);
			}else{
				area=cvContourArea(c,CV_WHOLE_SEQ);
			}

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

//Function for detecting the number of contour defects in an image
int detect_old(IplImage* img_8uc1,IplImage* img_8uc3, int use_accel) {
	CvMemStorage* storage = cvCreateMemStorage();
	CvSeq* first_contour = NULL;
	CvSeq* maxitem = NULL;
	double area = 0;
	double areamax = 0;
	int maxn=0;
	int value;
	int Nc = cvFindContours(img_8uc1,storage,&first_contour,sizeof(CvContour),CV_RETR_LIST);
	int n=0;
	int copy_size;

	if(Nc>0){
		for(CvSeq* c=first_contour; c!=NULL; c=c->h_next){
			if(use_accel){
				//select the total elements register register
				//This must be loaded with the number of elements in the contour.
				if(-1 == ioctl(fd_area_accel, SELECT_REG, ELEM_COUNT_REG)){
					perror("ioctl failed");
					return -1;
				}

				//write number of elements to register
				write(fd_area_accel, &(c->total), 4);
				copy_size = c->elem_size * c->first->count;

				//copy point sequence into first buffer
				memcpy(frame_buffer_1, c->first->data, copy_size);

				//start the accelerator
				if(-1 == ioctl(fd_area_accel, ACCEL_START)){
					perror("ioctl failed");
					return -1;
				}

				//select the upper word of the return register
				if(-1 == ioctl(fd_area_accel, SELECT_REG, RETURN_REG)){
					perror("ioctl failed");
					return -1;
				}
				read(fd_area_accel, &value, 4);
				area = (double)(-value);
			}else{
				area=cvContourArea(c,CV_WHOLE_SEQ );
			}

			if(area>areamax){
				areamax=area;
				maxitem=c;
				maxn=n;
			}
			n++;
		}
		CvMemStorage* storage3 = cvCreateMemStorage(0);

		if(areamax>5000){
			maxitem = cvApproxPoly(maxitem, sizeof(CvContour), storage3, CV_POLY_APPROX_DP, 10, 1);
			CvPoint pt0;

			CvMemStorage* storage1 = cvCreateMemStorage(0);
			CvMemStorage* storage2 = cvCreateMemStorage(0);
			CvSeq* ptseq = cvCreateSeq(CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),
					sizeof(CvPoint), storage1);
			CvSeq* hull;
			CvSeq* defects;

			for(int i = 0; i < maxitem->total; i++){
				CvPoint* p = CV_GET_SEQ_ELEM(CvPoint, maxitem, i);
				pt0.x = p->x;
				pt0.y = p->y;
				cvSeqPush(ptseq, &pt0);
			}
			hull = cvConvexHull2(ptseq, 0, CV_CLOCKWISE, 0);
			int hullcount = hull->total;

			//Determine the total number of defects

			defects= cvConvexityDefects(ptseq,hull,storage2);

			CvConvexityDefect* defectArray;  

			int j=0;  
			for(;defects;defects = defects->h_next){  
				nomdef = defects->total; //total number of detected defects

				if(nomdef == 0)  
					continue;  

				// Alloc memory for array of defects    
				defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*nomdef);  

				// Get defect set.  
				cvCvtSeqToArray(defects,defectArray, CV_WHOLE_SEQ);  

				// Draw marks for all the defects in the image.  
				if(DRAWING_ON){
					for(int i=0; i<nomdef; i++){   
						cvLine(img_8uc3, *(defectArray[i].start), *(defectArray[i].depth_point),CV_RGB(0,0,164),1, CV_AA, 0);  
						cvCircle(img_8uc3, *(defectArray[i].depth_point), 5, CV_RGB(164,0,0), 2, 8,0);  
						cvCircle(img_8uc3, *(defectArray[i].start), 5, CV_RGB(164,0,0), 2, 8,0);  
						cvLine(img_8uc3, *(defectArray[i].depth_point), *(defectArray[i].end),CV_RGB(0,0,164),1, CV_AA, 0);  
					} 

					char txt[]="0";
					txt[0]='0'+nomdef-1;
					CvFont font;
					cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 1.0, 1.0, 0, 1, CV_AA);
					cvPutText(img_8uc3, txt, cvPoint(50, 50), &font, cvScalar(0, 0, 128, 0)); 
				}
				j++;  

				// Free memory.         
				free(defectArray);  
			} 

			cvReleaseMemStorage( &storage );
			cvReleaseMemStorage( &storage1 );
			cvReleaseMemStorage( &storage2 );
			cvReleaseMemStorage( &storage3 );
		}
	}
	return 0;
}

