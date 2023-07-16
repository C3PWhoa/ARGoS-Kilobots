
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

/**
 * If the robot approaches a stationary robot at a distance less than desired,
 * it enters the 'ORBIT_NORMAL' state. Otherwise, it moves straight ahead.
 */
void orbit_forward() {
    printf("%d forward\n", kilo_uid);
    set_motion(LEFT);
}

/**
 * The robot is within the shape, it stops moving and transmits its message.
 */
void orbit_stop() {
    printf("%d stop\n", kilo_uid);
    set_motion(STOP);

}

void loop() {

    if (message_sent == 1) {
        message_sent = 0;

        set_color(RGB(0, 1, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
    }

    if (new_message == 1) {

        printf("new message == %d\n", new_message);
        new_message = 0;

        switch (current_motion) {
            case STOP:
                orbit_stop();
                break;
            case FORWARD:
                orbit_forward();
                break;
            default:
                break;
        }

    }
}

void message_rx(message_t *m, distance_measurement_t *d) {

    new_message = 1;
    distance = estimate_distance(d);
    current_motion = FORWARD;
}


message_t *message_tx() {
    return &message;
}

void message_tx_success() {
    // Set flag on message transmission.
    message_sent = 1;
}

int main() {
    kilo_init();
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_success;
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);

    return 0;
}


