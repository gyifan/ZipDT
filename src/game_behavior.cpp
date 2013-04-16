#include "game_behavior.h"
#include <sys/time.h>
#include <stdlib.h>
#include <ncurses.h>
#include <string.h>

//external global variables
extern int board[BOARD_HEIGHT][BOARD_WIDTH];
//extern int rows_cleared;
//extern int level;			//holds difficulty level
//extern int score;
extern struct game_stats stats;
extern int game_over;		//1=game ended. 0=still playing.
extern int board_changed;		//1= board changed, 0=board is same. redraw board if it changed, else do nothing.
extern double xComLoc;
extern double yComLoc;
extern struct current_piece piece;	//piece that user is controlling
extern struct timeval last_time;
extern struct timeval current_time;

//returns 0 if time not expired. returns 1 if time is expired.
int check_timer(){
	int new_usec; 
	int old_usec;

	gettimeofday(&current_time, NULL);
	new_usec = current_time.tv_sec * 1000000 + current_time.tv_usec;
	old_usec = last_time.tv_sec * 1000000 + last_time.tv_usec;

	if((new_usec - old_usec) >= get_delay())
		return 1;

	return 0;
}

void reset_timer(){
	gettimeofday(&last_time, NULL);
}

int get_delay(){
	int delay;
	if(stats.level < LEVEL_LIMIT){
		delay = BASE_DELAY + (stats.level * (MIN_DELAY - BASE_DELAY)/(LEVEL_LIMIT));
	}else
		delay = MIN_DELAY;
	return delay;
}

void init_game_stats(){
	stats.score = 0;
	stats.rows_cleared = 0;
	stats.level = 0;
	stats.data_a = 0;
	stats.data_b = 0;
	//stats.data_c = NULL;
	stats.data_d = 0;
	stats.data_e = 0;
	stats.data_f = 0;
	strcpy(stats.msg_a, "Execution time [us]:");
	strcpy(stats.msg_b, "Total frame throughput [fps]:");
	strcpy(stats.msg_c, "Direction:");
	strcpy(stats.msg_d, "Number of defects:");
	//stats.msg_c[0] = '\0';
	//stats.msg_d[0] = '\0';
	stats.msg_e[0] = '\0';
	stats.msg_f[0] = '\0';
}

//set all locations on the board to zero
void zero_board(){
	int i;
	int j;
	for(i=0; i < BOARD_HEIGHT; i++)
		for(j=0; j < BOARD_WIDTH; j++)
			board[i][j] = 0;
}

//randomly generates pieces 
int generate_piece(){	
	int retval = 0;
	int rand1 = (random() % NUM_PIECES) + 1; //determine piece type, vary from 1:7
	//int rand2; //will be used to calculate the rotation

	piece.x = BOARD_WIDTH/2 - 2; //try to center where pieces appear. -2 to account for offsets in pieces in header.
	piece.y = 0;
	piece.r = 0;
	piece.type = rand1;
	piece.r_limit = PIECE_R_LIMITS[rand1];

	get_blocks(piece.blocks, piece.x, piece.y, piece.type, piece.r);

	retval = check_collision(piece.blocks);
	place_piece();
	return retval;
}

//attempts to rotate piece clockwise. returns 0 on success, -1 on failure. May 
//fail if result is causes a collison.
int rotate_piece(){
	int r_old;
	int r_new;
	int retval = 0;
	int new_blocks[BLOCKS_IN_PIECE * 2];

	r_old = piece.r;
	r_new = (r_old + 1) % piece.r_limit;	

	remove_piece();
	get_blocks(new_blocks, piece.x, piece.y, piece.type, r_new);

	//test if any of piece's blocks are at invalid locations
	if(check_collision((int*)new_blocks)){
		retval = -1;
	}else{
	get_blocks(piece.blocks, piece.x, piece.y, piece.type, r_new);
	piece.r = r_new;
	}

	place_piece();

	return retval;
}

/*
Check if the any blocks of block array passed to the function  are in invalid 
locations on the board. 

Returns:
-1 = Collision present
0 = No collision

*/
int check_collision(int *blocks){
	int i;
	int x;
	int y;

	for(i=0; i < BLOCKS_IN_PIECE * 2; i+=2){
		x = blocks[i];
		y = blocks[i + 1];

		if(x < 0 || x >= BOARD_WIDTH || y < 0 || y >= BOARD_HEIGHT)
			return -1; //a piece is outside of the board! 

		if(EMPTY != board[y][x])// && CURRENT_BLOCK != board[y][x])
			return -1; //a space on board is not empty that block needs to fill
	}

	return 0;
}

/*
Copies the src blocks coordinates to the destination block array, adding the 
desired x and y offset. 
*/
void get_blocks(int *dest, int x_offset, int y_offset, int type, int rotation){
	int i;
	const int *src;

	switch(type){
	case BLOCK_O:
		src = PIECE_O_R0;
		break;	
	case BLOCK_I:
		switch(rotation){
		case 0:
			src = PIECE_I_R0;
			break;
		case 1:
			src = PIECE_I_R1;
			break;
		}
		break;
	case BLOCK_S:
		switch(rotation){
		case 0:
			src = PIECE_S_R0;
			break;
		case 1:
			src = PIECE_S_R1;
			break;
		}
		break;
	case BLOCK_Z:
		switch(rotation){
		case 0:
			src = PIECE_Z_R0;
			break;
		case 1:
			src = PIECE_Z_R1;
			break;
		}
		break;
	case BLOCK_J:
		switch(rotation){
		case 0:
			src = PIECE_J_R0;
			break;
		case 1:
			src = PIECE_J_R1;
			break;
		case 2:
			src = PIECE_J_R2;
			break;
		case 3:
			src = PIECE_J_R3;
			break;
		}
		break;
	case BLOCK_L:
		switch(rotation){
		case 0:
			src = PIECE_L_R0;
			break;
		case 1:
			src = PIECE_L_R1;
			break;
		case 2:
			src = PIECE_L_R2;
			break;
		case 3:
			src = PIECE_L_R3;
			break;
		}
		break;
	case BLOCK_T:
		switch(rotation){
		case 0:
			src = PIECE_T_R0;
			break;
		case 1:
			src = PIECE_T_R1;
			break;
		case 2:
			src = PIECE_T_R2;
			break;
		case 3:
			src = PIECE_T_R3;
			break;
		}
		break;
	}

	for(i=0; i < BLOCKS_IN_PIECE * 2; i+=2){
		dest[i] = src[i] + x_offset;
		dest[i+1] = src[i+1] + y_offset;
	}
}

void remove_piece(){
	int i;
	for(i=0; i < BLOCKS_IN_PIECE * 2; i+=2)
		board[piece.blocks[i + 1]][piece.blocks[i]] = EMPTY;
}

void place_piece(){
	int i;
	for(i=0; i < BLOCKS_IN_PIECE * 2; i+=2)
		board[piece.blocks[i + 1]][piece.blocks[i]] = piece.type;
}

//--Movement functions---------------------------------------------------

//control function called when user sends 'move down' input or timer is up
void handle_move_down(){
	//try to move down	
	if(move_down()){
		//failed to move down

		//scan board/remove lines/ give user points
		handle_rows();
		//create new piece
		if(generate_piece()){
			//GAME OVER!
			game_over = 1;
			return; //no timer reset so no need to pause
		}
	}
	reset_timer();
}

//drops piece straight down as far as it can go. generates a new piece if possible.
void handle_drop(){
	//mover her down. all the way down.
	while(!move_down()){};

	handle_rows();

	//check game over condition
	if(generate_piece()){
		game_over = 1;
		return;
	}
	reset_timer();
}

//scan each row to see if it is full. delete rows that are full and shift all 
//rows that are above down
void handle_rows(){
	int row;
	int col;
	int col_count;
	int row_count = 0;
	int new_rows = 0;
	int y;
	int x;

	//select row
	for(row = BOARD_HEIGHT - 1; row >= 0; row--){
		col_count = 0;
		//check each column
		for(col = 0; col < BOARD_WIDTH; col++){
			if(board[row][col] == EMPTY)
				break;
			col_count++;
		}

		//check if complete row
		if(col_count == BOARD_WIDTH){
			row_count++;

			//delete row
			for(col = 0; col < BOARD_WIDTH; col++)
				board[row][col] = EMPTY;

			//shift above rows down one
			for(y = row; y > 0; y--)
				for(x = 0; x < BOARD_WIDTH; x++)
					board[y][x]=board[y-1][x];

			row++;	//handle cases where shifting down and decrementing row 
			//cause row to be skipped!
		}

	}

	adjust_score(row_count);
	new_rows = stats.rows_cleared + row_count;

	//check if level needs to be adjusted here. This allows level to be initialized to 
	//a value that does not correspond to a score and not be reset on level adjustment.
	if(stats.rows_cleared/10 != new_rows/10)
		adjust_level();

	stats.rows_cleared = new_rows;
}

void adjust_level(){
	//stats.level = (stats.rows_cleared + 10)/ 10 - 1;
	stats.level++;
}

void adjust_score(int num_rows){
	stats.score += row_points[num_rows]*(stats.level+1);
}

/*
Attempt to move piece down one block. If succeeds the piece coordinates are 
adjusted. If fails, the piece is not moved.

Returns:
-1 on failure
0 success
*/
int move_down(){
	int retval;
	int new_blocks[BLOCKS_IN_PIECE * 2];

	get_blocks(new_blocks, piece.x, piece.y + 1, piece.type, piece.r);
	remove_piece();
	retval = check_collision(new_blocks);	//check if new location is invalid
	if(retval == 0){
		piece.y++;
		get_blocks(piece.blocks, piece.x, piece.y, piece.type, piece.r);
	}
	place_piece();
	return retval;

}

int move_left(){
	int retval;
	int new_blocks[BLOCKS_IN_PIECE * 2];

	get_blocks(new_blocks, piece.x - 1, piece.y, piece.type, piece.r);
	remove_piece();
	retval = check_collision(new_blocks);	//check if new location is invalid
	if(retval == 0){
		piece.x--;
		get_blocks(piece.blocks, piece.x, piece.y, piece.type, piece.r);
	}
	place_piece();
	return retval;
}

int move_right(){
	int retval;
	int new_blocks[BLOCKS_IN_PIECE * 2];

	get_blocks(new_blocks, piece.x + 1, piece.y, piece.type, piece.r);
	remove_piece();
	retval = check_collision(new_blocks);	//check if new location is invalid
	if(retval == 0){
		piece.x++;
		get_blocks(piece.blocks, piece.x, piece.y, piece.type, piece.r);
	}
	place_piece();
	return retval;
}

//--Display functions---------------------------------------------------------

//prints the board to the terminal in a readable format
void print_board(){
	int x;
	int y;

	printf("\n");
	for(y=0; y < BOARD_HEIGHT; y++){
		printf("|");
		for(x=0; x < BOARD_WIDTH; x++)
			printf("%d", board[y][x]);
		printf("|\n");
	}
	printf("\n");
}

void print_board_curses(){
	int x;
	int y;
	int block;

	clear();

	//Draw borders of gameboard----
	//redraw top of board
	y=BOARD_Y_OFFSET-1;
	for(x=BOARD_X_OFFSET-1; x <= BOARD_WIDTH*2 + BOARD_X_OFFSET; x++){
		mvaddch(y,x, WALL_CHAR_TOP);
	}
	
	//redraw bottom of board
	y=BOARD_Y_OFFSET + BOARD_HEIGHT - 2;	//compensate for 2 rows of game that will not be displayed
	for(x=BOARD_X_OFFSET-1; x <= BOARD_WIDTH*2 + BOARD_X_OFFSET; x++){
		mvaddch(y,x, WALL_CHAR_BOTTOM);
	}

	//redraw left side of board
	x=BOARD_X_OFFSET-1;	
	for(y=BOARD_Y_OFFSET-1; y <= BOARD_HEIGHT + BOARD_Y_OFFSET - 2; y++){
		mvaddch(y,x, WALL_CHAR_LEFT);
	}

	//redraw right side of board
	x=BOARD_X_OFFSET + BOARD_WIDTH*2;	
	for(y=BOARD_Y_OFFSET-1; y <= BOARD_HEIGHT + BOARD_Y_OFFSET - 2; y++){
		mvaddch(y,x, WALL_CHAR_RIGHT);
	}

	//Draw the blocks in the gameboard
	// 2 to hide top 2 rows per game behavior.
	for(y=2; y < BOARD_HEIGHT; y++){
		for(x=0; x < BOARD_WIDTH*2; x+=2){
			block = board[y][x/2];
			attron(COLOR_PAIR(block));
			mvaddch(y+BOARD_Y_OFFSET-2,x+BOARD_X_OFFSET,' ');
			mvaddch(y+BOARD_Y_OFFSET-2,x+BOARD_X_OFFSET+1,' ');
			attroff(COLOR_PAIR(block));
		}
	}

	//Display game stats
	x=STATS_X_OFFSET;
	y=STATS_Y_OFFSET;
	mvprintw(y+0,x,"Score:");
	mvprintw(y+1,x,"%d",stats.score);
	mvprintw(y+2,x,"Rows cleared:");
	mvprintw(y+3,x,"%d",stats.rows_cleared);
	mvprintw(y+4,x,"Level:");
	mvprintw(y+5,x,"%d",stats.level);

	//Print addition stat/debug information
	//ONLY PRINTS DATA IF MESSAGE IS ASSIGNED!!
	if(strnlen(stats.msg_a, MAX_MSG_LEN) != 0){
		mvprintw(y+6,x,stats.msg_a);
		mvprintw(y+7,x,"%d",stats.data_a);
	}	
	if(strnlen(stats.msg_b, MAX_MSG_LEN) != 0){
		mvprintw(y+8,x,stats.msg_b);
		mvprintw(y+9,x,"%d",stats.data_b);
	}
	if(strnlen(stats.msg_c, MAX_MSG_LEN) != 0){
		mvprintw(y+10,x,stats.msg_c);
		mvprintw(y+11,x,/*"%d",*/stats.data_c);
	}
	if(strnlen(stats.msg_d, MAX_MSG_LEN) != 0){
		mvprintw(y+12,x,stats.msg_d);
		mvprintw(y+13,x,"%d",stats.data_d);
	}
	if(strnlen(stats.msg_e, MAX_MSG_LEN) != 0){
		mvprintw(y+14,x,stats.msg_e);
		mvprintw(y+15,x,"%d",stats.data_e);
	}
	if(strnlen(stats.msg_f, MAX_MSG_LEN) != 0){
		mvprintw(y+14,x,stats.msg_f);
		mvprintw(y+15,x,"%d",stats.data_f);
	}
	
	//Print the center of mass indicator below the playing board
	//frame the indicator in a square
	//draw left edge
	for(y = BOARD_HEIGHT + BOARD_Y_OFFSET; y < BOARD_HEIGHT + BOARD_Y_OFFSET + BOARD_WIDTH + 2; y++){
		mvprintw(y, BOARD_X_OFFSET - 1, "]");
	}
	//draw right edge
	for(y = BOARD_HEIGHT + BOARD_Y_OFFSET; y < BOARD_HEIGHT + BOARD_Y_OFFSET + BOARD_WIDTH + 2; y++){
		mvprintw(y, 2*(BOARD_WIDTH) + BOARD_X_OFFSET, "[");
	}
	//draw top edge
	for(x = BOARD_X_OFFSET; x < 2*(BOARD_WIDTH + 1); x++){
		mvprintw(BOARD_HEIGHT + BOARD_Y_OFFSET, x, "v");
	}
	//draw bottom edge
	for(x = BOARD_X_OFFSET; x < 2*(BOARD_WIDTH + 1); x++){
		mvprintw(BOARD_HEIGHT + BOARD_Y_OFFSET + BOARD_WIDTH + 1, x, "^");
	}

	mvprintw(BOARD_HEIGHT + BOARD_Y_OFFSET + BOARD_WIDTH + 2, 0, "Tracked object center of mass.");
	
	//draw the indicator within the square frame
	mvprintw(BOARD_HEIGHT + BOARD_Y_OFFSET + yComLoc*BOARD_WIDTH + 1,
			2*(BOARD_WIDTH - xComLoc*BOARD_WIDTH) + BOARD_X_OFFSET, "0");


	refresh();
}
