#include "frame_grabber.h"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include "opencv/cv.h"
#include "main.h"

static int fd_video;
static struct v4l2_capability cap;
struct v4l2_fmtdesc fmtdesc;
struct v4l2_format fmt,fmtack;
struct v4l2_streamparm setfps;
struct v4l2_requestbuffers reqbuf;
struct v4l2_buffer buf;
enum v4l2_buf_type type;
char* yuyv_buffer;
char* frame_buffer[BUFF_COUNT];

pthread_mutex_t frame_mutex[BUFF_COUNT];

int has_frame[BUFF_COUNT];
extern struct capture_data capture_info;

struct buffer* buffers = NULL;

int init_capture_data(char* name){
	strncpy(capture_info.name, name, MAX_NAME_LEN);

	if(0 == strncmp(capture_info.name, CAPTURE_NAME_480, MAX_NAME_LEN)){
		capture_info.size = CAPTURE_SIZE_480;
		capture_info.dim.width = CAPTURE_WIDTH_480;
		capture_info.dim.height = CAPTURE_HEIGHT_480;
	}else if(0 == strncmp(capture_info.name, CAPTURE_NAME_720, MAX_NAME_LEN)){
		capture_info.size = CAPTURE_SIZE_720;
		capture_info.dim.width = CAPTURE_WIDTH_720;
		capture_info.dim.height = CAPTURE_HEIGHT_720;
	}else if(0 == strncmp(capture_info.name, CAPTURE_NAME_1080, MAX_NAME_LEN)){
		capture_info.size = CAPTURE_SIZE_1080;
		capture_info.dim.width = CAPTURE_WIDTH_1080;
		capture_info.dim.height = CAPTURE_HEIGHT_1080;
	}else
		return -1;	//error if an unsupported resolution name is provided
	return 0;	
}

//Be certain that this function is called in the parent proces...
int init_v4l2(void){
	int i;

	for(i = 0; i < BUFF_COUNT; i++)
		has_frame[i] = 0;

	if((fd_video = open(FILE_VIDEO, O_RDWR)) == -1){
		perror("init_v4l2: failed to open v4l interface");
		return -1;
	}

	if(ioctl(fd_video, VIDIOC_QUERYCAP, &cap) == -1){
		perror("init_v4l2: failed to ioctl device");
		return -1;
	}else{
		printf("driver:\t\t%s\n",cap.driver);
		printf("card:\t\t%s\n",cap.card);
		printf("bus_info:\t%s\n",cap.bus_info);
		printf("version:\t%d\n",cap.version);
		printf("capabilities:\t%x\n",cap.capabilities);

		if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE){
			printf("Device %s: supports capture.\n",FILE_VIDEO);
		}

		if((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING){
			printf("Device %s: supports streaming.\n",FILE_VIDEO);
		}
	}

	fmtdesc.index=0;
	fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	printf("Support format:\n");
	while(ioctl(fd_video,VIDIOC_ENUM_FMT,&fmtdesc)!=-1){
		printf("\t%d.%s\n",fmtdesc.index+1,fmtdesc.description);
		fmtdesc.index++;
	}

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.height = capture_info.dim.height;
	fmt.fmt.pix.width = capture_info.dim.width;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

	if(ioctl(fd_video, VIDIOC_S_FMT, &fmt) == -1){
		perror("init_v4l2: Unable to set format");
		return -1;
	}
	if(ioctl(fd_video, VIDIOC_G_FMT, &fmt) == -1){
		perror("init_v4l2: Unable to get format");
		return -1;
	}
	else{
		printf("fmt.type:\t\t%d\n",fmt.type);
		printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF,(fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
		printf("pix.height:\t\t%d\n",fmt.fmt.pix.height);
		printf("pix.width:\t\t%d\n",fmt.fmt.pix.width);
		printf("pix.field:\t\t%d\n",fmt.fmt.pix.field);
	}
	//set fps
	setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps.parm.capture.timeperframe.numerator = 10;
	setfps.parm.capture.timeperframe.denominator = 10;

	//Allocate buffers

	//buffer request
	memset(&reqbuf, 0, sizeof(reqbuf));
	reqbuf.count=1;	//just use the single yuyv_buffer
	reqbuf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;

	reqbuf.memory=V4L2_MEMORY_MMAP;
	//reqbuf.memory=V4L2_MEMORY_USERPTR;

	if(ioctl(fd_video,VIDIOC_REQBUFS,&reqbuf)==-1){
		perror("init_v4l2: request for buffers error");
		return -1;
	}
	
	buf.index = 0;
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	//query buffers
	if(ioctl (fd_video, VIDIOC_QUERYBUF, &buf) == -1){
		perror("init_v4l2: query buffer error");
		return -1;
	}

	//map
	if(MAP_FAILED == (yuyv_buffer = (char*)mmap(NULL,buf.length,PROT_READ|PROT_WRITE, MAP_SHARED, fd_video, buf.m.offset))){
		perror("init_v4l2: buffer map error");
		return -1;
	}

	//allocate buffers to store RGB888 frames
	for(i = 0; i < BUFF_COUNT; i++)
		if(NULL == (frame_buffer[i] = (char*)calloc(1, capture_info.size))){
			perror("init_v4l2: frame_buffer calloc failed");
			return -1;
		}

	//initialize mutexes
	for(i = 0; i < BUFF_COUNT; i++)
		frame_mutex[i] = PTHREAD_MUTEX_INITIALIZER;

	return 0;
}

int v4l2_grab(void){
	buf.index = 0;	//only have 1 buffer mmapped so is index 0
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	ioctl(fd_video,VIDIOC_QBUF, &buf);
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(fd_video, VIDIOC_STREAMON, &type);
	ioctl(fd_video, VIDIOC_DQBUF, &buf);

	return 0;
}

int yuyv_2_rgb888(char* dst_ptr){
	int i,j;
	int temp0;
	int temp1;
	unsigned char y1,y2,u1,v1;
	unsigned char y3,y4,u2,v2;
	unsigned char y5,y6,u3,v3;
	unsigned char y7,y8,u4,v4;
	int r1,g1,b1,r2,g2,b2;
	int r3,g3,b3,r4,g4,b4;
	int r5,g5,b5,r6,g6,b6;
	int r7,g7,b7,r8,g8,b8;
	char* pointer;
	int height = capture_info.dim.height;
	int width = capture_info.dim.width;

	pointer = yuyv_buffer;

	for(i=0;i<height;i++){
		for(j=0;j<width/2;j+=4){
			temp0 = (i*width/2+j)*4;
			temp1 = (i*width/2+j)*6;

			y1 = *(pointer + temp0);
			u1 = *(pointer + temp0 + 1);
			y2 = *(pointer + temp0 + 2);
			v1 = *(pointer + temp0 + 3);
			y3 = *(pointer + temp0 + 4);
			u2 = *(pointer + temp0 + 5);
			y4 = *(pointer + temp0 + 6);
			v2 = *(pointer + temp0 + 7);

			y5 = *(pointer + temp0 + 8);
			u3 = *(pointer + temp0 + 9);
			y6 = *(pointer + temp0 + 10);
			v3 = *(pointer + temp0 + 11);
			y7 = *(pointer + temp0 + 12);
			u4 = *(pointer + temp0 + 13);
			y8 = *(pointer + temp0 + 14);
			v4 = *(pointer + temp0 + 15);

			r1 = y1 + 1.042 * (v1 - 128);
			g1 = y1 - 0.34414 * (u1 - 128) - 0.71414 * (v1 - 128);
			b1 = y1 + 1.772 * (u1 - 128);
			r2 = y2 + 1.042 * (v1 - 128);
			g2 = y2 - 0.34414 * (u1 - 128) - 0.71414 * (v1 - 128);
			b2 = y2 + 1.772 * (u1 - 128);
			r3 = y3 + 1.042 * (v2 - 128);
			g3 = y3 - 0.34414 * (u2 - 128) - 0.71414 * (v2 - 128);
			b3 = y3 + 1.772 * (u2 - 128);
			r4 = y4 + 1.042 * (v2 - 128);
			g4 = y4 - 0.34414 * (u2 - 128) - 0.71414 * (v2 - 128);
			b4 = y4 + 1.772 * (u2 - 128);

			r5 = y5 + 1.042 * (v3 - 128);
			g5 = y5 - 0.34414 * (u3 - 128) - 0.71414 * (v3 - 128);
			b5 = y5 + 1.772 * (u3 - 128);
			r6 = y6 + 1.042 * (v3 - 128);
			g6 = y6 - 0.34414 * (u3 - 128) - 0.71414 * (v3 - 128);
			b6 = y6 + 1.772 * (u3 - 128);
			r7 = y7 + 1.042 * (v4 - 128);
			g7 = y7 - 0.34414 * (u4 - 128) - 0.71414 * (v4 - 128);
			b7 = y7 + 1.772 * (u4 - 128);
			r8 = y8 + 1.042 * (v4 - 128);
			g8 = y8 - 0.34414 * (u4 - 128) - 0.71414 * (v4 - 128);
			b8 = y8 + 1.772 * (u4 - 128);
			
			//ensure that values are in an acceptable range
			r1 &= 255; g1 &= 255; b1 &= 255;
			r2 &= 255; g2 &= 255; b2 &= 255;
			r3 &= 255; g3 &= 255; b3 &= 255;
			r4 &= 255; g4 &= 255; b4 &= 255;

			r5 &= 255; g5 &= 255; b5 &= 255;
			r6 &= 255; g6 &= 255; b6 &= 255;
			r7 &= 255; g7 &= 255; b7 &= 255;
			r8 &= 255; g8 &= 255; b8 &= 255;

			*(dst_ptr + temp1) = (unsigned char)b1;
			*(dst_ptr + temp1 + 1) = (unsigned char)g1;
			*(dst_ptr + temp1 + 2) = (unsigned char)r1;
			*(dst_ptr + temp1 + 3) = (unsigned char)b2;
			*(dst_ptr + temp1 + 4) = (unsigned char)g2;
			*(dst_ptr + temp1 + 5) = (unsigned char)r2;
			*(dst_ptr + temp1 + 6) = (unsigned char)b3;
			*(dst_ptr + temp1 + 7) = (unsigned char)g3;
			*(dst_ptr + temp1 + 8) = (unsigned char)r3;
			*(dst_ptr + temp1 + 9) = (unsigned char)b4;
			*(dst_ptr + temp1 + 10) = (unsigned char)g4;
			*(dst_ptr + temp1 + 11) = (unsigned char)r4;

			*(dst_ptr + temp1 + 12) = (unsigned char)b5;
			*(dst_ptr + temp1 + 13) = (unsigned char)g5;
			*(dst_ptr + temp1 + 14) = (unsigned char)r5;
			*(dst_ptr + temp1 + 15) = (unsigned char)b6;
			*(dst_ptr + temp1 + 16) = (unsigned char)g6;
			*(dst_ptr + temp1 + 17) = (unsigned char)r6;
			*(dst_ptr + temp1 + 18) = (unsigned char)b7;
			*(dst_ptr + temp1 + 19) = (unsigned char)g7;
			*(dst_ptr + temp1 + 20) = (unsigned char)r7;
			*(dst_ptr + temp1 + 21) = (unsigned char)b8;
			*(dst_ptr + temp1 + 22) = (unsigned char)g8;
			*(dst_ptr + temp1 + 23) = (unsigned char)r8;

			//*(unsigned int*)(dst_ptr + temp1) = word0;
			//*(unsigned int*)(dst_ptr + temp1 + 4) = word1;
			//*(unsigned int*)(dst_ptr + temp1 + 8) = word2;
		}
	}
	return 0;
}

//locks the next available frame buffer and returns the buffer index
int lock_frame_buffer(int current_frame){
	int next_frame = current_frame;
	int have_lock = 0;

	//select next open frame buffer
	while(!have_lock){
		next_frame = ((next_frame+1) % BUFF_COUNT);

		if(0 == pthread_mutex_trylock(&frame_mutex[next_frame])){
			have_lock = 1;
			current_frame = next_frame;
		}
	}
	return next_frame;
}

//when called, this function finds the next available frame buffer and fills
//the buffer with an RGB888 frame.
void* capture_frame(void* param){
#ifdef USE_MULTI_THREAD_CAPTURE
	for(;;){
#endif
	static int current_buffer = 0;
	v4l2_grab();
	current_buffer = lock_frame_buffer(current_buffer);
	yuyv_2_rgb888(frame_buffer[current_buffer]);
	has_frame[current_buffer] = 1;
	pthread_mutex_unlock(&frame_mutex[current_buffer]);
#ifdef USE_MULTI_THREAD_CAPTURE
	}
#endif
	return NULL;
}

IplImage* capImage = NULL;
//This function will only operate correctly if *only* ONE image is to be captured
//and held at a time by a function that calls v4lQueryFrame. If multiple calls to
//this function are made, all resulting pointers will reference the SAME image.
//If multiple calls are required, be certain to clone the resulting image!
IplImage* v4lQueryFrame(){
	static int current_buffer = 0;
	static int valid_frame = 0;

	//if holding a frame buffer currently, release it to be filled again. 
	//Set has_frame to false to not re-use the same frame.
	if(valid_frame){
		has_frame[current_buffer] = 0;
		valid_frame = 0;
		pthread_mutex_unlock(&frame_mutex[current_buffer]);
	}

	//scan the frame buffers for a frame
	while(!valid_frame){
		current_buffer = lock_frame_buffer(current_buffer);
		//check if buffer has been initialized
		if(0 == has_frame[current_buffer]){
			//if buffer is empty, release it and try again
			pthread_mutex_unlock(&frame_mutex[current_buffer]);
		}else{
			//frame buffer has a frame. use it.
			valid_frame = 1;
		}
	}

	//avoid memory leak incurred by not deallocating image header, make capImage static
	if(NULL == capImage)
		capImage = cvCreateImageHeader(capture_info.dim, IPL_DEPTH_8U, 3);

	capImage->nSize = sizeof(IplImage);
	capImage->nChannels = 3;
	capImage->depth = IPL_DEPTH_8U;
	capImage->dataOrder = 0;	//0 = interleaved
	capImage->origin = 0;
	capImage->width = capture_info.dim.width;
	capImage->height = capture_info.dim.height;
	capImage->roi = NULL;
	capImage->maskROI = NULL;
	capImage->imageId = NULL;
	capImage->tileInfo = NULL;
	capImage->imageSize = capture_info.size;
	capImage->imageData = frame_buffer[current_buffer];	
	capImage->widthStep = capImage->width * 3;
	capImage->imageDataOrigin = NULL;
	
	return capImage;
}

