#ifndef __H_MAIN_H__
#define __H_MAIN_H__

#define	USE_V4L_READS
#define USE_MULTI_THREAD_CAPTURE

//---------------------------------------------------------------------------
//define the parameter options to be used with getopt(3)
//see manpage for getopt(3) for more details on parameter string
#define	ARG					":"
#define	OPTARG				"::"

#define	START_LEVEL			'l'
#define	ACCEL_SET_ALL		'a'
#define ACCEL_SET_AREA		'c'	//use contour area accelerator
#define CAPTURE_RESOLUTION	'r'
#define CALIB_FRAMES		'f'
#define USE_DRAWING			'G'
#define KEYBOARD_INPUT		'K'
#define SET_DEBUG_MODE		'D'

//could not come up with an elegand way to write this.
//must append character options above to list manually.
#define PARAM_STRING "+l:acr:f:GKDh"

#endif