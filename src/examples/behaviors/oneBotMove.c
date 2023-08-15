
#include <kilolib.h>
#include <stdio.h>

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

// Threshold of error tolerance during robot positioning within the shape.
#define THRESHOLD 3


message_t message;
int message_sent = 0;
int new_message = 0;
int current_motion = STOP;
int distance;


void set_motion(int new_motion) {
    // Only take an action if the motion is being changed.
    if (current_motion != new_motion) {
        current_motion = new_motion;

        if (current_motion == STOP) {
            set_motors(0, 0);
        } else if (current_motion == FORWARD) {
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        } else if (current_motion == LEFT) {
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        } else if (current_motion == RIGHT) {
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

void setup() {

}



void loop() {

    if (kilo_ticks <= 67){
        set_motion(LEFT);
    }
}




int main() {
    kilo_init();
    kilo_start(setup, loop);

    return 0;
}


