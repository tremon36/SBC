#ifndef _data_structures_robot_h
#define _data_structures_robot_h

/* Necessary data strutures for robot operation*/

typedef struct robot_state {

    float* position;        // two element array
    int rotation;           // rotation is always integer
    int falling_risk;       // risk of falling if continuing to go in the direction
    int pause;              // indicates pause state

} robot_state_t;


#endif