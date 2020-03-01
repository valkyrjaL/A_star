/*=================================================================
 *
 * planner.c
 * Andrew ID: chingtil
 * 2019/02/10
 *
 *=================================================================*/
 
#include <math.h>
#include "mex.h"
#include <stdlib.h>
#include <string.h>
#include "minHeap.h"
#include <stdio.h>
#include <algorithm>
// #include <chrono>
// using namespace std;


/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


int temp = 0;

static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           int robotposeX,
            int robotposeY,
            int goalposeX,
            int goalposeY,
            char *p_actionX,
            char *p_actionY
		   )
{
	// auto start = chrono::steady_clock::now();

    //8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};    
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
	double cost[NUMOFDIRS] = {1.4, 1.0, 1.4, 1.0, 1.0, 1.4, 1.0, 1.4};
    
    printf("call=%d\n", temp);
    temp = temp+1;
	
	// /*
	MinHeap openList;
	int cell_size = x_size * y_size;
	cell* cellDetails = new cell[cell_size];
	// Initial cellDetails
	for(int i=0; i<cell_size; i++){
		// cellDetails[i].f = FLT_MAX;
		cellDetails[i].g = FLT_MAX;
		// cellDetails[i].h = FLT_MAX;
		cellDetails[i].closed = false;
	}
	// Initial start state and add start into open list
	int robotIndex = GETMAPINDEX(robotposeX,robotposeY,x_size,y_size);
	int targetIndex = GETMAPINDEX(goalposeX,goalposeY,x_size,y_size);
	cellDetails[robotIndex].f = 0.0;
	cellDetails[robotIndex].g = 0.0;
	// cellDetails[robotIndex].h = 0.0;
	cellDetails[robotIndex].x = robotposeX;
	cellDetails[robotIndex].y = robotposeY;
	cellDetails[robotIndex].predecessor = nullptr;
	openList.add(cellDetails[robotIndex]);
	// printf("%f", openList.peek().f);
	int expandX, expandY, expandIndex;
    
	while((openList.heapCount() != 0) && (cellDetails[targetIndex].closed == false)){
		cell tempCell = openList.pop();
		tempCell.closed = true;
		expandX = tempCell.x;
		expandY = tempCell.y;
		expandIndex = GETMAPINDEX(expandX,expandY,x_size,y_size);
		
		for(int dir = 0; dir < NUMOFDIRS; dir++)
		{
			int newx = expandX + dX[dir];
			int newy = expandY + dY[dir];
			int successor = GETMAPINDEX(newx,newy,x_size,y_size);
    
			if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){//if valid              
				if ((int)map[successor] == 0){ //if free
					if(!cellDetails[successor].closed){
						double g_temp = cellDetails[expandIndex].g + cost[dir];
						if(cellDetails[successor].g > g_temp){
							cellDetails[successor].g = g_temp;
							// Euclidean Distance
							// cellDetails[successor].h = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) 
														// + (newy-goalposeY)*(newy-goalposeY)));
							// Manhattan Distance
							// cellDetails[successor].h = abs((newx-goalposeX)) + abs(newy-goalposeY);
							// cellDetails[successor].f = cellDetails[successor].g + cellDetails[successor].h;
							cellDetails[successor].f = cellDetails[successor].g + (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
							cellDetails[successor].x = newx;
							cellDetails[successor].y = newy;
							cellDetails[successor].predecessor = &cellDetails[expandIndex];
							openList.add(cellDetails[successor]);
						}
					}
				}
			}
		}
	}
	
	// backtracking
	cell backCell = cellDetails[targetIndex];
	cell *startCell = &cellDetails[robotIndex];
	while(backCell.predecessor != startCell){
		backCell = *backCell.predecessor;
	}
	// printf("x: %d", backCell.x - robotposeX);
	// printf("y: %d", backCell.y - robotposeY);
	*p_actionX = backCell.x - robotposeX;
	*p_actionY = backCell.y - robotposeY;
	
	if(cellDetails != nullptr){
		delete[] cellDetails;
		cellDetails = nullptr;
	}
	// auto end = chrono::steady_clock::now();
	// printf("time: %d ms ", chrono::duration_cast<chrono::milliseconds>(end - start));
	// */
	
    //printf("robot: %d %d; ", robotposeX, robotposeY);
    //printf("goal: %d %d;", goalposeX, goalposeY);
    
	/*
	//for now greedily move towards the target, 
	//but this is where you can put your planner 
	double mindisttotarget = 1000000;
	for(int dir = 0; dir < NUMOFDIRS; dir++)
	{
    	   int newx = robotposeX + dX[dir];
     	   int newy = robotposeY + dY[dir];
    
    	   if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){
               
              if ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] == 0){ //if free
                double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
				if(disttotarget < mindisttotarget){
                  mindisttotarget = disttotarget;
                  *p_actionX = dX[dir];
                  *p_actionY = dY[dir];
                }
              }
    	   }
	}
	auto end = chrono::steady_clock::now();
	printf("time: %d us ", chrono::duration_cast<chrono::microseconds>(end - start));
	*/
    //printf("action: %d %d; \n", *p_actionX, *p_actionY);

	
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector <x,y> for the robot pose
//3rd is a row vector <x,y> for the target pose
//plhs should contain output parameters (1): 
//1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 2.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    int goalposeX = (int)goalposeV[0];
    int goalposeY = (int)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxINT8_CLASS, mxREAL); 
    char* action_ptr = (char*)  mxGetPr(ACTION_OUT);
            
    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY, &action_ptr[0], &action_ptr[1]); 
    return;
    
}



/*--------------------------------------*/

MinHeap::MinHeap(){
	capacity = 100;
	size = 0;
	items = new cell[capacity];
}

MinHeap::~MinHeap(){
	if(items != nullptr){
		delete[] items;
		items = nullptr;
	}
}

void MinHeap::swap(int indexOne, int indexTwo){
	cell temp = items[indexOne];
	items[indexOne] = items[indexTwo];
	items[indexTwo] = temp;
}

/* If the array was full, increase the array with capacity.*/
void MinHeap::ensureExtraCapacity(){
	if(size == capacity){
		//copy items to a new array
		cell *temp = new cell[capacity * 2];
		std::copy(items, items + capacity, temp);
		delete[] items;
		items = temp;
		capacity *= 2;
	}
}

	
void MinHeap::heapifyUp(){
	int newIndex = size-1;
	while(hasParent(newIndex) && (parent(newIndex).f > items[newIndex].f)){
		swap(getParentIndex(newIndex), newIndex);
		newIndex = getParentIndex(newIndex);
	}
}

void MinHeap::heapifyDown(){
	int index = 0;
	while(hasLeftChild(index)){
		int smallerChildIndex = getLeftChildIndex(index);
		if(hasRightChild(index) && (leftChild(index).f > rightChild(index).f)){
			smallerChildIndex = getRightChildIndex(index);
		}
		if(items[index].f > items[smallerChildIndex].f){
			swap(index, smallerChildIndex);
			index = smallerChildIndex;
		}else{
			break;
		}
	}
}

cell MinHeap::pop(){
	// if(size == 0) return -1;	// throw exception
	cell item = items[0];
	items[0] = items[size-1];
	size --;
	heapifyDown();
	return item;
}

void MinHeap::add(cell item){
	ensureExtraCapacity();
	items[size] = item;
	size++;
	heapifyUp();
}

/* Peek the first node without removing it.*/
cell MinHeap::peek(){
	// if(size == 0) return -1;	// throw exception
	return items[0];
}
