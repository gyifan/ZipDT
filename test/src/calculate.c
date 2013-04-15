#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ROT_FILE "log_rot.txt"
#define LR_FILE "log_lr.txt"

int main(int argc, char *argv[]){
	int type;
	FILE *fd;
	
	// Take in input from the command line
	if(argc != 2){
		printf("usage: calculate type(0/1)\n");
		exit(-1);
	}else{
		/* 0 for rotation, 1 for left/right */
		type = atoi(argv[1]); 
		
		/* Open the file */
		if(type){
			fd = fopen(LR_FILE, "r");
			printf("opened %s\n", LR_FILE);
		}else {
			fd = fopen(ROT_FILE, "r");
			printf("opened %s\n", ROT_FILE);
		}
	}
	int dt[1000];
	char *name[1000];
	
	int counter;
	int defect;
	int average_delay;

	char temp[80];
	int t;
	
	int curtime;
	int prevtime;
	char *curname;
	curname = malloc(10);
	char *prevname;
	prevname = malloc(10);
	
	switch (type){
		case 1: {
		
		/* If the leading char changes (left, right), measures the difference.
		 * And increases the counter. Store the difference in an array.
		 */	
			counter = 0;
			defect = 0;
			prevtime = 0;
			strcpy(prevname, "ok");
			while(NULL!=fgets(temp, 80, fd)){
				strcpy(curname, strtok(temp, ":"));
				curtime = atoi(strtok(NULL, ":"));
				if(strcmp(curname, "Rotation")){
					if(0!=strcmp(curname, prevname)){
						dt[counter] = curtime-prevtime;
						name[counter] = malloc(10);
						strcpy(name[counter], curname);
						counter ++;
					}
				} else defect ++;
				strcpy(prevname,curname);
				prevtime = curtime;
			}
			break;
		}
		case 0: {
		
		/* Count all rotations occurred. 
		 * Record any defect happened.
		 */
			counter = 0;
			defect = 0;
			prevtime = 0;
			while(NULL!=fgets(temp, 80, fd)){
				curname = strtok(temp, ":");
				curtime = atoi(strtok(NULL, ":"));
				if(strcmp(curname, "Rotation"))
					defect ++;
				else {
					name[counter] = calloc(10, sizeof(char));
					name[counter] = curname;
					dt[counter] = curtime-prevtime;
					counter ++;
				}
				prevtime = curtime;
			}
			break;
		}
		default: printf("Unrecognizable Mode: rot or lr");
	}	
	average_delay = 0;
	for(t=0; t<counter; t++){
		average_delay += dt[t];
	}
	average_delay = average_delay/counter;
	
	//Print out the name and time list.
	
	for(t=0;t<counter; t++){
		printf("%s: %d\n", name[t], dt[t]);	
	}
	printf("Average Delay is: %d\n", average_delay);
	return 0;
}
