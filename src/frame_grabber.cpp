#include "frame_grabber.h"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
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

sem_t frame_sem[BUFF_COUNT];
pthread_mutex_t frame_mutex[BUFF_COUNT];

int has_frame[BUFF_COUNT];
int capture_pipe_fd[2];
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

	//initialize semaphores
	for(i = 0; i < BUFF_COUNT; i++)
		if(sem_init(&frame_sem[i], 1, 1)){
			perror("init_v4l2: sem_init failed");
			return -1;
		}

//	for(i)

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
	unsigned char y1,y2,u,v;
	int r1,g1,b1,r2,g2,b2;
	char* pointer;
	int height = capture_info.dim.height;
	int width = capture_info.dim.width;

	pointer = yuyv_buffer;

	for(i=0;i<height;i++){
		for(j=0;j<width/2;j++){
			y1 = *( pointer + (i*width/2+j)*4);
			u  = *( pointer + (i*width/2+j)*4 + 1);
			y2 = *( pointer + (i*width/2+j)*4 + 2);
			v  = *( pointer + (i*width/2+j)*4 + 3);

			r1 = y1 + 1.042*(v-128);
			g1 = y1 - 0.34414*(u-128) - 0.71414*(v-128);
			b1 = y1 + 1.772*(u-128);

			r2 = y2 + 1.042*(v-128);
			g2 = y2 - 0.34414*(u-128) - 0.71414*(v-128);
			b2 = y2 + 1.772*(u-128);

			if(r1>255)
				r1 = 255;
			else if(r1<0)
				r1 = 0;

			if(b1>255)
				b1 = 255;
			else if(b1<0)
				b1 = 0;

			if(g1>255)
				g1 = 255;
			else if(g1<0)
				g1 = 0;

			if(r2>255)
				r2 = 255;
			else if(r2<0)
				r2 = 0;

			if(b2>255)
				b2 = 255;
			else if(b2<0)
				b2 = 0;

			if(g2>255)
				g2 = 255;
			else if(g2<0)
				g2 = 0;
			
			*(dst_ptr + ((i)*width/2+j)*6    ) = (unsigned char)b1;
			*(dst_ptr + ((i)*width/2+j)*6 + 1) = (unsigned char)g1;
			*(dst_ptr + ((i)*width/2+j)*6 + 2) = (unsigned char)r1;
			*(dst_ptr + ((i)*width/2+j)*6 + 3) = (unsigned char)b2;
			*(dst_ptr + ((i)*width/2+j)*6 + 4) = (unsigned char)g2;
			*(dst_ptr + ((i)*width/2+j)*6 + 5) = (unsigned char)r2;

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

		if(0 == sem_trywait(&frame_sem[next_frame])){
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
	sem_post(&frame_sem[current_buffer]);
#ifdef USE_MULTI_THREAD_CAPTURE
	}
#endif
	return NULL;
}

//This function will only operate correctly if *only* ONE image is to be captured
//and held at a time by a function that calls v4lQueryFrame. If multiple calls to
//this function are made, all resulting pointers will reference the SAME image.
//If multiple calls are required, be certain to clone the resulting image!
IplImage* v4lQueryFrame(){
	static IplImage* capImage = NULL;
	static int current_buffer = 0;
	static int valid_frame = 0;

	//if holding a frame buffer currently, release it to be filled again. 
	//Set has_frame to false to not re-use the same frame.
	if(valid_frame){
		has_frame[current_buffer] = 0;
		valid_frame = 0;
		sem_post(&frame_sem[current_buffer]);
	}

	//scan the frame buffers for a frame
	while(!valid_frame){
		current_buffer = lock_frame_buffer(current_buffer);
		//check if buffer has been initialized
		if(0 == has_frame[current_buffer]){
			//if buffer is empty, release it and try again
			sem_post(&frame_sem[current_buffer]);
		}else{
			//frame buffer has a frame. use it.
			valid_frame = 1;
		}
	}

	//avoid memory leak incurred by not deallocating image header, make rawImage static
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

