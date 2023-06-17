//
// Created by frederic turchet on 08/06/2023.
//
#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>

// Constants for motion handling function.
#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

// Constants for orbit control.
#define TOO_CLOSE_DISTANCE 60
#define DESIRED_DISTANCE 70

static const uint8_t NB_BOT = 4;

typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
    RANDOM_WALK,
    MOVE_LIGHT,
} orbit_state_t;
orbit_state_t states[NB_BOT];

message_t message;

int current_motion = STOP;
int new_message = 0, message_sent = 0;
int speaker_id, received_state;

int range_dist[NB_BOT];


int min_received_dist(int tab_dist[], int nb_bot) {
    int return_min = 200;
    for (int i = 0; i < nb_bot; ++i) {
        if (tab_dist[i] < return_min) {
            return_min = tab_dist[i];
        }
    }
    return return_min;
}

// Function to handle motion.
void set_motion(int new_motion)
{
    // Only take an action if the motion is being changed.
    if (current_motion != new_motion)
    {
        current_motion = new_motion;

        if (current_motion == STOP)
        {
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD)
        {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == LEFT)
        {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == RIGHT)
        {
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

void setup(){
    for (int i = 0; i < 3; ++i) {
        states[i] = ORBIT_STOP;
    }
    for (int i = 0; i < NB_BOT; ++i) {
        states[i] = ORBIT_FORWARD;
    }

    for (int i = 0; i < NB_BOT; ++i) {
        range_dist[i] = 200;
    }

    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.crc = message_crc(&message);
}


void orbit_normal(){
    int min = min_received_dist(range_dist, NB_BOT);
    if (min < TOO_CLOSE_DISTANCE){
        states[kilo_uid] = ORBIT_TOOCLOSE;
    } else {
        if (min < DESIRED_DISTANCE){
            set_motion(LEFT);
        } else {
            set_motion(RIGHT);
        }
    }
}

void orbit_tooClose(){
    int min = min_received_dist(range_dist, NB_BOT);
    if (min < TOO_CLOSE_DISTANCE){
        set_motion(FORWARD);
    } else {
        states[kilo_uid] = ORBIT_NORMAL;
    }
}

void orbit_forward(){
    int min = min_received_dist(range_dist, NB_BOT);

    if (min > DESIRED_DISTANCE){
        set_motion(FORWARD);
    } else {
        states[kilo_uid] = ORBIT_NORMAL;
    }
}


void loop(){
    if (message_sent == 1){
        message_sent = 0;
    }

    if (new_message == 1) {
        new_message = 0;

        switch (states[kilo_uid]) {
            case ORBIT_TOOCLOSE:
                orbit_tooClose();
                break;
            case ORBIT_NORMAL:
                orbit_normal();
                break;
            case ORBIT_FORWARD:
                orbit_forward();
                break;
            default:
                break;
        }
    }
}

void message_rx(message_t *m, distance_measurement_t *d){
    new_message = 1;
    speaker_id = (*m).data[0];
    received_state = (*m).data[1];
    if(received_state == ORBIT_STOP){
        range_dist[speaker_id] = estimate_distance(d);
    }
}

message_t *message_tx(){
    return &message;
}

void message_tx_success(){
    message_sent = 1;
}

int main(){
    kilo_init();
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_success;
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);

    return 0;
}

