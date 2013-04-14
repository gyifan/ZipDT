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
#include "opencv/cxmisc.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv/cxmisc.h"

#include "main.h"
#include "contour_area.h"
#include "game_behavior.h"
#include "frame_grabber.h"
#include "frame_processing.h"
#include "accelerators.h"

int DEBUG_MODE = 0;
int TESTING_MODE = 0;
int USE_KEYBOARD = 0;
int DRAWING_ON = 0;
int USE_V4L_CAPTURE = 1;

//global variables
int board[BOARD_HEIGHT][BOARD_WIDTH];
int input_char; 	//character that useir has entered
int last_input;
int use_accelerators = USE_ACCEL_NONE;

int game_over = 0;	//1=game ended. 0=still playing.
int board_changed = 1;	//1= board changed, 0=board is same. redraw board if it changed, else do nothing.
int fps = 0;
struct current_piece piece;	//piece that user is controlling
struct timeval last_time;
struct timeval current_time;
WINDOW *win;
struct game_stats stats;
struct capture_data capture_info;

//Delay calculation variables
time_t before;
time_t after;
clock_t t;
struct timeval timA;
struct timeval timB;
struct timeval timC;
struct timeval timD;
struct timeval timE;

CvCapture* capture = NULL;

void init_curses_mode(){
	//begin curses mode
	win = initscr();
	start_color();

	//initialize color pairs
	init_pair(0,COLOR_BLACK,COLOR_BLACK);
	init_pair(1,COLOR_BLACK,COLOR_WHITE);
	init_pair(2,COLOR_BLACK,COLOR_CYAN);//,COLOR_BLACK);
	init_pair(3,COLOR_BLACK,COLOR_YELLOW);//,COLOR_BLACK);
	init_pair(4,COLOR_BLACK,COLOR_MAGENTA);//,COLOR_BLACK);
	init_pair(5,COLOR_BLACK,COLOR_GREEN);//,COLOR_BLACK);
	init_pair(6,COLOR_BLACK,COLOR_RED);//,COLOR_BLACK);
	init_pair(7,COLOR_BLACK,COLOR_WHITE);//,COLOR_BLACK);

	curs_set(0);	//hide cursor
	cbreak();	//allow reading chars without carriage return
	noecho();	//turn off input echo
	nodelay(win, true); //turn getch into non-blocking call
}

void usage(char* cmd){
	printf("\nusage: %s [options]\n", cmd);
	printf("\n");
	printf("<HARDWARE ACCELERATORS>\n");
	printf("If no accelerator parameter is passed, no accelerators are utilized.\n");
	printf("\t-a\tuse all hardware accelerators\n");
	printf("\t-c\tuse cvFindContourArea hardware accelerator\n");
	printf("\t-e\tuse cvErode hardware accelerator\n");
	printf("\n<CAPTURE RESOLUTION>\n");
	printf("If no resolution parameter is passed, the program defaults to 640x480 frame captures.\n");
	printf("\t-r <RESOLUTION>\n");
	printf("\t\t\t480\tcapture frames at 640x480\n");
	printf("\t\t\t720\tcapture frames at 1280x720\n");
	printf("\t\t\t1080\tcapture frames at 1920x1080\n");
	printf("\n<FRAME PROCESSING>\n");
	printf("\n<GAME BEHAVIOR>\n");
	printf("Change how game statistics are initialized.\n");
	printf("\t-l <LEVEL>\tSet the starting level of the game.\n");
	printf("\n<MISC>\n");
	printf("\t-N\tUse standard opencv capture to acquire frames. Disables lower-level V4L capture.\n");
	printf("\t-G\tEnable High GUI displays.\n");
	printf("\t-K\tDisable image processing for input and use i,j,k,l,<SPACE> to control game.\n");
	printf("\t-D\tDebug mode. Display timing information and other data.\n");
	printf("\t-M\tTesting mode. Display timing and response information.");
	printf("\n");
}

int main(int argc, char** argv){
	int calib_frames = 100;
	int use_accelerators = USE_ACCEL_NONE;
	int temp;
	int param;
	extern char* optarg;
	extern int optind, opterr, optopt;
	extern int nomdef;
#ifdef USE_MULTI_THREAD_CAPTURE
	pthread_t capture_thread;
#endif

	init_game_stats();

	init_capture_data(CAPTURE_NAME_480);

	//parse input arguments
	while(-1 != (param = getopt(argc, argv,PARAM_STRING))){
		switch(param){
			case START_LEVEL:
				stats.level = atoi(optarg);
				break;

			case ACCEL_SET_ALL:
				use_accelerators |= USE_ACCEL_ALL;
				break;
			case ACCEL_SET_AREA:
				use_accelerators |= USE_ACCEL_AREA;
				break;
			case ACCEL_SET_ERODE:
				use_accelerators |= USE_ACCEL_ERODE;
				break;

			case CAPTURE_RESOLUTION:
				if(init_capture_data(optarg)){
					printf("\nunrecognized capture resolution \"%s\"\n", optarg);
					usage(argv[0]);
					return -1;
				}
				break;

			case NO_V4L_CAPTURE:
				USE_V4L_CAPTURE = 0;
				break;

			case SET_DEBUG_MODE:
				printf("\nGame has been disabled for debug mode.\n\n");
				DEBUG_MODE = 1;
				break;

			case SET_TESTING_MODE:
				printf("\nTesting mode has been enabled.\n\n");
				TESTING_MODE = 1;
				break;

			case USE_DRAWING:
				DRAWING_ON = 1;
				break;

			case KEYBOARD_INPUT:
				USE_KEYBOARD = 1;
				break;

			case 'h':
				usage(argv[0]);
				return 0;
			default:
				printf("\nunrecognized option \"%c\"\n", param);
				usage(argv[0]);
				return -1;
		}
	}

	if(init_accel(use_accelerators)){
		printf("Failed to initialize accelerator(s)\n");
		return -1;
	}

	//Allocate and initialize the CvCapture Structure
	//A parameter of -1 indicates uses the first available camera
	//In this case, we only expect one camera to be attached so a parameter of -1 is acceptable
	if(!USE_V4L_CAPTURE){
		capture = cvCaptureFromCAM(-1);
		cvQueryFrame(capture);
		//Use the queried frame in order to determine an appropriate size for newly created windows
		capture_info.dim.height = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
		capture_info.dim.width = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	}

	//do not initialize capture if it is not being used.
	if(!USE_KEYBOARD){

		if(USE_V4L_CAPTURE){
			if(init_v4l2()){
				printf("Failed to initialize V4l.\n");
				return -1;
			}
#ifdef USE_MULTI_THREAD_CAPTURE //multi-thread capture relies on V4L reads
			if(pthread_create(&capture_thread, NULL, capture_frame, NULL)){
				perror("main: pthread_create failed");
				return -1;
			}
#endif
		}
	}

	//disable curses display in debug mode
	if(!DEBUG_MODE && !TESTING_MODE)
		init_curses_mode();

	//do not initialize frame processing if it is not being used.
	if(!USE_KEYBOARD)
		init_frame_processing(calib_frames);

	srand(time(NULL));	//make random seed for generate_piece()

	//initialize board
	zero_board();

	//create initial piece for user
	generate_piece();

	//initialize timer
	reset_timer();

	//BEGIN THE GAME LOOP
	while(!game_over){
		if(USE_KEYBOARD){
			input_char = getch();
		}else{
			//read input using image proccessing.
			gettimeofday(&timA,NULL);
			input_char = get_input();
			gettimeofday(&timB,NULL);
			timersub(&timB, &timA, &timC);

			temp = timC.tv_usec + timC.tv_sec * 1000000;
			stats.data_a = temp;	//total microseconds
			stats.data_b = 1000000/temp;	//fps
			stats.data_d = nomdef;
			if(DEBUG_MODE)
				printf("Total time[us] %d\n", temp);
			if(TESTING_MODE){
				if(input_char == INPUT_RIGHT){
					gettimeofday(&timB,NULL);
					timersub(&timB, &timA, &timC);
					temp = timC.tv_usec + timC.tv_sec * 1000000;
					printf("Right Input[us] %d\n", temp);
				}
				if(input_char == INPUT_LEFT){
					gettimeofday(&timB,NULL);
					timersub(&timB, &timA, &timC);
					temp = timC.tv_usec + timC.tv_sec * 1000000;
					printf("Left Input[us] %d\n", temp);
				}
				if(input_char == INPUT_SPIN_CW){
					gettimeofday(&timB,NULL);
					timersub(&timB, &timA, &timC);
					temp = timE.tv_usec + timE.tv_sec * 1000000;
					printf("Left Input[us] %d\n", temp);
				}
			}
		}

		//disable game behavior in debug mode to allow image processing functions
		//to be called forever
		if(!DEBUG_MODE && !TESTING_MODE){
			//Map specific motion to an existing control in the game
			if(input_char == INPUT_RIGHT){
				strcpy(stats.data_c, "Right");
				if(0 == move_right()){
					board_changed = 1;
				}
			}else if(input_char == INPUT_LEFT){
				strcpy(stats.data_c, "Left");
				if(0 == move_left()){
					board_changed = 1;
				}
			}else if(input_char == INPUT_SPIN_CW){
				if(0 == rotate_piece()){
					board_changed = 1;
				}	
			}else if(input_char == INPUT_DOWN){
				handle_move_down();	//THIS WAS COMMENTED OUT. strange behavior ensues, re-comment-out
				board_changed = 1;
			}else if(input_char == INPUT_DROP){
				handle_drop();
				board_changed = 1;
			}

			//check if time delay is finished for moving piece down
			if(check_timer()){
				handle_move_down();
				board_changed = 1;
			}

			if(board_changed){
				print_board_curses();
				board_changed = 0;
			}
		}
	}

	if(!USE_V4L_CAPTURE){
		cvReleaseCapture(&capture);
	}

	if(!DEBUG_MODE){
		nodelay(win, false); //turn getch into blocking call to wait for input
		endwin();	//end curses mode	
	}

	return 0;
}

