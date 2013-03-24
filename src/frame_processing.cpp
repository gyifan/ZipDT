
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

IplImage* rawImage = NULL;
IplImage* yuvImage = NULL;
IplImage* ImaskCodeBook = NULL;
IplImage* ImaskCodeBookCC = NULL;
IplImage* ImaskCodeBookCCInv = NULL;
IplImage* im_gray = NULL;

//The variables below are used to map game controls
int turncount = 0;
bool turnaround = false;
bool pieceRight = false;
bool pieceLeft = false;
bool pieceDown = false;
int rightcount = 0;
int leftcount = 0;
int downcount = 0;
int delaystart = 0;

//nomdef - Number of defects in the produced image contour
int nomdef = 0;
int nframesToLearnBG = 100;
static const double pi = 3.14159265358979323846;
//Used as a delay variable in order to prevent constant spinning of game pieces
int fingerDelay = 0;

//Delay calculation variables
int nloop = 1;

//Number of frame features that will be analyzed with optical flow		
//I currently designated the number of frame to be 100, but this number can be increased for improved accuracy
//It should be noted that increasing the number of features will result in a lower processing rate
const int MAX_FEATURES = 100;

//This array will contain the features found in the first frame
CvPoint2D32f frame1_features[MAX_FEATURES];

//This array will contain the features found in the second frame
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

	nframesToLearnBG = calib_frames;

	allocateOnDemand(&rawImage, capture_info.dim, IPL_DEPTH_8U, 3);
	allocateOnDemand(&yuvImage, capture_info.dim, IPL_DEPTH_8U, 3);

	allocateOnDemand(&ImaskCodeBook, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&ImaskCodeBookCC, capture_info.dim, IPL_DEPTH_8U, 1);
	allocateOnDemand(&ImaskCodeBookCCInv, capture_info.dim, IPL_DEPTH_8U, 1);

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

#ifndef	USE_V4L_READS
		rawImage = cvQueryFrame(capture);
#else
#ifndef USE_MULTI_THREAD_CAPTURE
		capture_frame((void*)NULL); //if not multi-threaded must capture an image b4 query
#endif
		rawImage = v4lQueryFrame();
#endif

#ifdef USE_V4L_READS
		cvCvtColor(rawImage, yuvImage, CV_RGB2YCrCb);//YUV For codebook method. V4L query returns RGB format
#else
		cvCvtColor(rawImage, yuvImage, CV_BGR2YCrCb);//YUV For codebook method
#endif
		if(nframes < nframesToLearnBG){
			cvBGCodeBookUpdate(model, yuvImage);
		}else if(nframes == nframesToLearnBG)
			cvBGCodeBookClearStale(model, model->t/2);
	}
	cvCopy(ImaskCodeBook,ImaskCodeBookCC);	

	if(DRAWING_ON){
		cvNamedWindow("Foreground", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Foreground",480,0);
		cvNamedWindow("Contours", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Contours", 640,480);
		cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Optical Flow", 480, 480);
	}
}

char get_input(){
	int sumDirection = 0;
	int sumvect = 0;
	int number_of_features;
	int i = 0;
	char input_char = INPUT_NONE;
	CvPoint p,q;
	//The Optical Flow window is created to visualize the output of the optical flow algorithm
	//The size of this winodw is automatically adjusted in order to match the previously determined window width and height

	if(DEBUG_MODE){
		printf("------------start of get input-------------\n");
		gettimeofday(&timevalA, NULL);
	}

#ifndef	USE_V4L_READS
	rawImage = cvQueryFrame(capture);
#else
#ifndef USE_MULTI_THREAD_CAPTURE
	capture_frame(); //if not multi-threaded must capture an image b4 query
#endif
	rawImage = v4lQueryFrame();
#endif

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("capture frame time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	if(NULL == rawImage) 
		return INPUT_NONE;	//error condition?

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

#ifdef USE_V4L_READS
	cvCvtColor(rawImage, yuvImage, CV_RGB2YCrCb);//YUV For codebook method. V4L query returns RGB format
#else
	cvCvtColor(rawImage, yuvImage, CV_BGR2YCrCb);//YUV For codebook method
#endif

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("cvt RGB2YUV time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	if(NULL == yuvImage)
		return INPUT_NONE;

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	// Find foreground by codebook method
	cvBGCodeBookDiff(model, yuvImage, ImaskCodeBookCC);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("cvBGCodeBookDiff time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	if(NULL == ImaskCodeBookCC)
		return INPUT_NONE;

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	cvSegmentFGMask(ImaskCodeBookCC);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("cvSegmentFGMask time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	if(NULL == ImaskCodeBookCC)
		return INPUT_NONE;

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	cvCopy(rawImage, contour_frame);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("cvCopy time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	if(use_accelerators & USE_ACCEL_AREA){
		if(detect(ImaskCodeBookCC, contour_frame, 1)){
			printf("detect failed\n");
		}		
	}else{
		if(detect(ImaskCodeBookCC, contour_frame, 0)){
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

	if(DEBUG_MODE){
		gettimeofday(&timevalA, NULL);
	}

	cvCvtColor(rawImage,im_gray,CV_RGB2GRAY);

	if(DEBUG_MODE){
		gettimeofday(&timevalB, NULL);
		timersub(&timevalB, &timevalA, &timevalC);
		printf("cvt RGB2GRAY time[us] = %d\n", timevalC.tv_sec * 1000000 + timevalC.tv_usec);
	}

	cvCopy(im_gray, frame1);
	cvCopy(im_gray, frame1_1C);

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
			eig_image,		//PARAM IGNORED
			temp_image,		//PARAM IGNORED
			frame1_features,	//array to hold corners
			&number_of_features,	//number of corners
			0.05,	//quality
			5,	//min euclidean dist between corners
			ImaskCodeBookCC,	//ROI mask
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

		sumDirection = 0;

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

			sumvect = p.x-q.x;

			if(sumvect < 0){
				if((-(sumvect) > X_DELTA_MIN) && (-(sumvect) < X_DELTA_MAX))
					sumDirection += sumvect;
			}else{
				if((sumvect > X_DELTA_MIN) && (sumvect < X_DELTA_MAX))
					sumDirection += sumvect;
			}
			/*
			   if(((p.x-q.x) > X_DELTA_MIN || (-(p.x-q.x)) > X_DELTA_MIN)
			   && (((p.x - q.x) < X_DELTA_MAX) || ((-(p.x - q.x)) < X_DELTA_MAX))){
			   sumDirection += p.x - q.x;
			   }
			   */

			//sumDirection += q.x - p.x;
		}

		if(DRAWING_ON){
			cvShowImage("Foreground", ImaskCodeBookCC);
			cvShowImage("Optical Flow",frame1);
			cvShowImage("Contours", contour_frame);
			cvWaitKey(1); //cause high gui to finish pending highgui operations. (allow images to be displayed)
		}
		if(DEBUG_MODE)
			printf("X_DIR_SUM = %d\n", sumDirection);

		//control logic starts here			
		if(sumDirection < 0){
			if(-(sumDirection) > X_DIRECTION_THRESHOLD){
				pieceRight = false;
				pieceLeft = true;
			}else{
				pieceRight = false;
				pieceLeft = false;
			}
		}else{
			if(sumDirection > X_DIRECTION_THRESHOLD){
				pieceRight = true;
				pieceLeft = false;
			}else{
				pieceRight = false;
				pieceLeft = false;
			}
		}
	}
	if(DRAWING_ON){
		//Troubleshooting text for motion detection
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 1.0, 1.0, 0, 1, CV_AA);
		cvPutText(frame1, motionindic, cvPoint(50, 50), &font, cvScalar(0, 0, 128, 0)); 
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

