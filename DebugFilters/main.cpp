/*
 * main.cpp
 *
 *  Created on: May 10, 2015
 *      Author: papigers
 */
#include <stdlib.h>
#include <stdio.h>

extern int gateDebug( int argc, char** argv );
extern int boxDebug( int argc, char** argv );

int main(int argc, char** argv){
	if(argc<2){
		printf("must enter mission\n");
	}
	else{
		int mission = atoi(argv[1]);
		if(mission == 1)
			return gateDebug(argc, argv);
		else if(mission == 4)
			return boxDebug(argc, argv);
		else if (mission == 0){
			gateDebug(argc,argv);
			boxDebug(argc,argv);
		}
	}
}


