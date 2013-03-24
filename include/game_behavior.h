#ifndef __H_GAME_BEHAVIOR_H__
#define __H_GAME_BEHAVIOR_H__

// timing and level data ------------------------------------------------
#define BOARD_HEIGHT	22	//only 20 should be visible
#define BOARD_WIDTH	10
#define BASE_DELAY	1000000	//microseconds
#define MIN_DELAY	125000	//microseconds
#define LEVEL_LIMIT	10	//After this level the delay will not decrease

// game display defines -------------------------------------------------
#define	BOARD_X_OFFSET		2
#define	BOARD_Y_OFFSET		2
#define WALL_CHAR_TOP		'v'
#define WALL_CHAR_BOTTOM	'^'
#define WALL_CHAR_LEFT		']'
#define WALL_CHAR_RIGHT		'['

#define STATS_X_OFFSET	BOARD_X_OFFSET + BOARD_WIDTH*2 + 2
#define STATS_Y_OFFSET	BOARD_Y_OFFSET - 1	//align stats with top wall of board

#define NUM_STATS_LINES	18	//a line is included for message and data of each statistic
#define MAX_MSG_LEN	50

struct game_stats{
	int score;
	int rows_cleared;
	int level;
	//somewhat dirty way to add extra messages to output display.
	//This could be cleaned-up but not worth it for now...
	char msg_a[MAX_MSG_LEN];
	char msg_b[MAX_MSG_LEN];
	char msg_c[MAX_MSG_LEN];
	char msg_d[MAX_MSG_LEN];
	char msg_e[MAX_MSG_LEN];
	char msg_f[MAX_MSG_LEN];
	int data_a;
	int data_b;
	int data_c;
	int data_d;
	int data_e;
	int data_f;
};

// point and score data -------------------------------------------------
const int row_points[5] = {
	0,
	40,
	100,
	300,
	1200
};

// Input defines --------------------------------------------------------
//Keyboard inputs for original version
#define INPUT_NONE		'z'
#define INPUT_LEFT		'j'
#define INPUT_RIGHT		'l'
#define INPUT_DOWN		'k'
#define INPUT_DROP		' '
#define INPUT_SPIN_CW	'i'
//#define INPUT_SPIN_CCW	0	//unused
#define INPUT_PAUSE		'p'
#define INPUT_QUIT		'q'

// Piece and block defines ----------------------------------------------
#define BLOCKS_IN_PIECE	4
#define NUM_PIECES	7

//The numbers that code for each type of piece. May be used to vary color.
#define EMPTY		0
#define BLOCK_O		1       //square
#define BLOCK_S		2       //s piece
#define BLOCK_Z		3       //z piece
#define BLOCK_I		4       //Bar piece
#define BLOCK_J		5       //Left facing L
#define BLOCK_L		6       //Right facing L
#define BLOCK_T		7       //T piece
#define CURRENT_BLOCK	8

//The number of rotation states for each type of piece
#define PIECE_O_R_COUNT 1
#define PIECE_I_R_COUNT	2
#define PIECE_S_R_COUNT	2
#define PIECE_Z_R_COUNT	2
#define PIECE_J_R_COUNT	4
#define PIECE_L_R_COUNT	4
#define PIECE_T_R_COUNT	4

//Indexed list of rotation limits. indexed by piece type.
const int PIECE_R_LIMITS[8] = {
	0,
	PIECE_O_R_COUNT,
	PIECE_I_R_COUNT,
	PIECE_S_R_COUNT,
	PIECE_Z_R_COUNT,
	PIECE_J_R_COUNT,
	PIECE_L_R_COUNT,
	PIECE_T_R_COUNT
};

//------Define the shape of each piece in each rotation state-----------------

/*
0 1 2 3
0 . . . .
1 . # # .
2 . # # .
3 . . . .
*/
const int PIECE_O_R0[BLOCKS_IN_PIECE * 2] = {1,1,2,1,1,2,2,2};

//--------------------------------------------------------------

/*
0 1 2 3
0 . . . .
1 . . . .
2 # # # #
3 . . . .
*/
const int PIECE_I_R0[BLOCKS_IN_PIECE * 2] = {0,2,1,2,2,2,3,2};

/*
0 1 2 3
0 . # . .
1 . # . .
2 . # . .
3 . # . .
*/
const int PIECE_I_R1[BLOCKS_IN_PIECE * 2] = {1,0,1,1,1,2,1,3};

//--------------------------------------------------------------

/*
0 1 2 3
0 . . . .
1 # # . .
2 . # # .
3 . . . .
*/
const int PIECE_Z_R0[BLOCKS_IN_PIECE * 2] = {0,1,1,1,1,2,2,2};

/*
0 1 2 3
0 . . # .
1 . # # .
2 . # . .
3 . . . .
*/
const int PIECE_Z_R1[BLOCKS_IN_PIECE * 2] = {2,0,1,1,2,1,1,2};

//--------------------------------------------------------------

/*
0 1 2 3
0 . . . .
1 . . # #
2 . # # .
3 . . . .
*/
const int PIECE_S_R0[BLOCKS_IN_PIECE * 2] = {2,1,3,1,1,2,2,2};

/*
0 1 2 3
0 . # . .
1 . # # .
2 . . # .
3 . . . .
*/
const int PIECE_S_R1[BLOCKS_IN_PIECE * 2] = {1,0,1,1,2,1,2,2};

//--------------------------------------------------------------

/* 
0 1 2 3
0 . . . .
1 # . . .
2 # # # .
3 . . . .
*/
const int PIECE_J_R0[BLOCKS_IN_PIECE * 2] = {0,1,0,2,1,2,2,2};
/*
0 1 2 3
0 . . . .
1 . # # .
2 . # . .
3 . # . .
*/
const int PIECE_J_R1[BLOCKS_IN_PIECE * 2] = {1,1,2,1,1,2,1,3};
/*
0 1 2 3
0 . . . .
1 . . . .
2 # # # .
3 . . # .
*/
const int PIECE_J_R2[BLOCKS_IN_PIECE * 2] = {0,2,1,2,2,2,2,3};
/*
0 1 2 3
0 . . . .
1 . # . .
2 . # . .
3 # # . .
*/
const int PIECE_J_R3[BLOCKS_IN_PIECE * 2] = {1,1,1,2,1,3,0,3};

//--------------------------------------------------------------

/*
0 1 2 3
0 . . . .
1 . . # .
2 # # # .
3 . . . .
*/
const int PIECE_L_R0[BLOCKS_IN_PIECE * 2] = {2,1,2,2,1,2,0,2};

/*
0 1 2 3
0 . . . .
1 . # . .
2 . # . .
3 . # # .
*/
const int PIECE_L_R1[BLOCKS_IN_PIECE * 2] = {1,1,1,2,1,3,2,3};

/*
0 1 2 3
0 . . . .
1 . . . .
2 # # # .
3 # . . .
*/
const int PIECE_L_R2[BLOCKS_IN_PIECE * 2] = {2,2,1,2,0,2,0,3};

/*
0 1 2 3
0 . . . .
1 # # . .
2 . # . .
3 . # . .
*/
const int PIECE_L_R3[BLOCKS_IN_PIECE * 2] = {0,1,1,1,1,2,1,3};

//--------------------------------------------------------------

/*
0 1 2 3
0 . . . .
1 . # . .
2 # # # .
3 . . . .
*/
const int PIECE_T_R0[BLOCKS_IN_PIECE * 2] = {1,1,0,2,1,2,2,2};

/*
0 1 2 3
0 . . . .
1 . # . .
2 . # # .
3 . # . .
*/
const int PIECE_T_R1[BLOCKS_IN_PIECE * 2] = {1,1,1,2,2,2,1,3};

/*
0 1 2 3
0 . . . .
1 . . . .
2 # # # .
3 . # . .
*/
const int PIECE_T_R2[BLOCKS_IN_PIECE * 2] = {0,2,1,2,2,2,1,3};

/*
0 1 2 3
0 . . . .
1 . # . .
2 # # . .
3 . # . .
*/
const int PIECE_T_R3[BLOCKS_IN_PIECE * 2] = {0,2,1,1,1,2,1,3};

//--------------------------------------------------------------

//function declarations
void zero_board();
int generate_piece();
int rotate_piece();
int check_collision(int* new_blocks);
void print_board();
void print_board_curses();
void get_blocks(int *dest, int x_offset, int y_offset, int type, int rotation);
void remove_piece();
void place_piece();
int move_down();
int move_left();
int move_right();
void reset_timer();
int check_timer();
int get_delay();
void handle_move_down();
void handle_drop();
void handle_rows();
void adjust_score(int num_rows);
void adjust_level();
void init_game_stats();

//structure containing information about the piece that the user may control
struct current_piece{
	int type;
	int blocks[BLOCKS_IN_PIECE * 2];
	int r;
	int r_limit;
	int x;	//monitor where the user has moved the piece. These may be negative due to offsets in pieces!
	int y;
};

#endif
