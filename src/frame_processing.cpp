
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
#include "game_behavior.h"
#include "frame_grabber.h"
#include "accelerators.h"
#include "frame_processing.h"

extern int DEBUG_MODE;
extern int DRAWING_ON;
extern int use_accelerators;
extern int USE_V4L_CAPTURE;

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
void* frame_buffer_1;

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
	if(use_accel != USE_ACCEL_NONE){

		//open contour area accelerator device file
		if(use_accel & USE_ACCEL_AREA){
			if(-1 == (fd_area_accel = open(DEV_PATH, O_RDWR | O_SYNC))){
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

	}
	return 0;
}

// currently this function ensures all images are allocated and creates the initial 
// background model.
void init_frame_processing(int calib_frames){
	int nframes;

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
		cvMoveWindow("Detected Skin",480,0);
		cvNamedWindow("Contours", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Contours", 640,480);
		cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Optical Flow", 480, 480);
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

	if(!USE_V4L_CAPTURE)
		rawImage = cvQueryFrame(capture);
	else{
#ifndef USE_MULTI_THREAD_CAPTURE
		capture_frame((void*)NULL); //if not multi-threaded must capture an image b4 query
#endif
		//rawImage = v4lQueryFrame();
		grayImage = v4lQueryFrame();
	}

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("capture frame time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	if(NULL == skinImage){
		return INPUT_NONE;	//error condition?
	}

	cvCopy(grayImage->maskROI, skinImage);

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	//use the morphological 'open' operation to remove small foreground noise.
	cvErode(skinImage, skinImage, NULL, 1);
	cvDilate(skinImage, skinImage, NULL, 1);
	//use the morphological 'close' operation to remove small background noise.
	cvDilate(skinImage, skinImage, NULL, 1);
	cvErode(skinImage, skinImage, NULL, 1);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("open and close frame time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	//reset the contour frame image.
	memset(contour_frame->imageData, 0, contour_frame->imageSize);
	cvCopy(skinImage, tmp_skinImage);

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	if(use_accelerators & USE_ACCEL_AREA){
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
	
	cvCopy(grayImage, frame1);
	cvCopy(grayImage, frame1_1C);

	if(NULL == frame1)
		return INPUT_NONE;
	if(NULL == frame1_1C)
		return INPUT_NONE;

	number_of_features = MAX_FEATURES;

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	cvGoodFeaturesToTrack(
			frame1_1C,		//input 8U or 32F image
			NULL,//eig_image,		//PARAM IGNORED
			NULL,//temp_image,		//PARAM IGNORED
			frame1_features,	//array to hold corners
			&number_of_features,	//number of corners
			0.001,	//quality
			0,	//min euclidean dist between corners
			//ImaskCodeBookCC,	//ROI mask
			//skinImage,	//ROI mask
			skinImage,
			5,		//block size
			0,		//0 = dont use harris, 1 = use harris corner detector
			0.04);	//parameter of harris corner detector (if its being used)

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("cvGoodFeaturesToTrack time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	//Optical flow motherload
	// frame1_1C: first frame with the known features.
	// frame2_1C: second frame used to detect the features found in frame1
	// "pyramid1" and "pyramid2" are workspace for the algorithm.
	// frame1_features: features from the first frame.
	// frame2_features: the locations of the features found in frame1, as found in the second frame.

	if(frame2_1C != NULL){
		if(DEBUG_MODE){
			gettimeofday(&timevalA, NULL);
		}

		cvCalcOpticalFlowPyrLK(
				frame1_1C, //previous image
				frame2_1C, //current image
				pyramid1, 
				pyramid2, 
				frame1_features, //previous image corners
				frame2_features, //current image corners
				number_of_features, //number of features
				cvSize(3,3), //size of pyramid window
				5, //max pyramid level to use //was 5
				optical_flow_found_feature, 
				optical_flow_feature_error, 
				optical_flow_termination_criteria, 
				0);

		if(DEBUG_MODE){
			gettimeofday(&timevalB, NULL);
			timersub(&timevalB, &timevalA, &timevalC);
			printf("cvCalcOpticalFlowPyrLK time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
		}

		sum_x = 0;
		sum_y = 0;

		//Draw lines for indication of optical flow vectors between frames
		//A line is drawn for each shared feature found between frames
		for(int i = 0; i < number_of_features; i++){
			//If there is no feature found, the continue to the next feature
			if(optical_flow_found_feature[i] == 0){
				continue;
			}
			//check error???
			//	if(optical_flow_feature_error[i] == 0)
			//		continue;

			//Declare variables used for line creation
			int line_thickness;				line_thickness = 1;
			CvScalar line_color;			line_color = CV_RGB(255,0,0);
			//Declare x and y coordinates for the same feature found in both frames
			//p is used for the first frame while q is used for the second frame
			p.x = (int) frame2_features[i].x;
			p.y = (int) frame2_features[i].y;
			q.x = (int) frame1_features[i].x;
			q.y = (int) frame1_features[i].y;

			diff_x = p.x-q.x;
			diff_y = p.y-q.y;
			temp_x = abs(diff_x);
			temp_y = abs(diff_y);

			if(DRAWING_ON){
				//Calculate the hypotenuse and angle between the same feature in 2 separate frames
				double angle;		angle = atan2((double) p.y - q.y, (double) p.x - q.x);
				double hypotenuse;	hypotenuse = sqrt((p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x));
				//Lengthen the size of the hypotenuse to make it more visible in the produced window
				q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
				q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
				cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);
				p.x = (int) (q.x + 9 * cos(angle + pi / 4));
				p.y = (int) (q.y + 9 * sin(angle + pi / 4));
				cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);
				p.x = (int) (q.x + 9 * cos(angle - pi / 4));
				p.y = (int) (q.y + 9 * sin(angle - pi / 4));
				cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);
			}

			//Only filter out max values of other coordinate to remove random 'big' noise
			if(temp_x > X_DELTA_MIN && temp_x < X_DELTA_MAX && temp_y < Y_DELTA_MAX){
				sum_x += diff_x;
			}

			if(temp_y > Y_DELTA_MIN && temp_y < Y_DELTA_MAX && temp_x < X_DELTA_MAX){
				sum_y += diff_y;
			}
		}

		if(DRAWING_ON){
			cvShowImage("Detected Skin", skinImage);
			//cvShowImage("Optical Flow",frame1);
			cvShowImage("Contours", contour_frame);
			cvWaitKey(1); //cause high gui to finish pending highgui operations. (allow images to be displayed)
		}

		if(DEBUG_MODE){
			printf("X_DIR_SUM = %d\n", sum_x);
			printf("Y_DIR_SUM = %d\n", sum_y);
		}

		
/*
		if(DEBUG_MODE){
			printf("X_DELTA_COM0 = %d\n", com_x_delta[0]);
			printf("Y_DELTA_COM0 = %d\n", com_y_delta[0]);
		}
*/

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
		
		
	}

	allocateOnDemand(&frame2_1C, capture_info.dim, IPL_DEPTH_8U, 1);
	//store the current frame to be used next itteration with optical flow
	cvCopy(frame1_1C, frame2_1C);
	//copy 'current frame' features into 'last frame' features
	//This could be faster if features array was made 2-D and addressed using a variable and
	//modulo addressing to identify current and last frame features(wouldn't need a loop, just ptr change)...
	for(i = 0; i < MAX_FEATURES; i++)
		memcpy(&frame2_features[i], &frame1_features[i], sizeof(CvPoint2D32f));


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

