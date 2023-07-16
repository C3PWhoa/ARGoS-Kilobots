
#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

// Constants for light following.
#define THRESH_LO 300
#define THRESH_HI 600

// Threshold of error tolerance during robot positioning within the shape.
#define THRESHOLD 3

static const int NB_BOT = 54;
static const uint8_t TOOCLOSE_DISTANCE = 40;
static const uint8_t DESIRED_DISTANCE = 65;
static const uint8_t TOOFAR_DISTANCE = 80;

typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
    MOVE_LIGHT,
    WAIT_TOGO,
} orbit_state_t;

message_t message;
int message_sent = 0;
int new_message = 0;
int current_motion = STOP;
int speaker_id;
int last_ticks_update;
int new_id[NB_BOT] = {[0 ... 11] = -1};
int range_dist[NB_BOT] = {[0 ... 11] = 200};
int received_state;
int received_newID;
int current_light = 0, received_one_mess = 0;
uint32_t last_state_update;

int nb_bot_form = 3;
int bot_in_form[NB_BOT] = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,
                           0, 0, 0, 0, 0, 0,0, 0,0, 0, 0,
                           0, 0, 0, 0, 0, 0,0, 0,0, 0, 0,
                           0, 0, 0, 0, 0, 0,0, 0,0};

orbit_state_t state = ORBIT_FORWARD;
orbit_state_t states[NB_BOT];



//[new_id]:{n1, d1, n2, d2} where 0 < ni < N_BOT and 'di' equal's distance between this and ni

int forme[NB_BOT][4] = {
        {1, 65, 2, 65},
        {0, 65, 2, 92},
        {0, 65, 1, 92},
        {1, 65, 2, 65},
        {2, 65, 3, 92},
        {3, 65, 4, 65},
        {4, 65, 5, 92},
        {6, 65, 5, 65},
        {5, 65, 7, 92},
        {8, 65, 7, 65},
        {7, 65, 6, 92},
        {10, 65, 6, 65},
        {4, 65, 6, 92},
        {6, 65, 12, 65},
        {8, 65, 9, 92},
        {14, 65, 9, 65},
        {12, 65, 13, 92},
        {16, 65, 13, 65},
        {10, 65, 11, 92},
        {18, 65, 11, 65},
        {18, 65, 19, 92},
        {19, 65, 20, 65},
        {20, 65, 21, 92},
        {21, 65, 22, 65},
        {22, 65, 23, 92},
        {23, 65, 24, 65},
        {24, 65, 25, 92},
        {26, 65, 25, 65},
        {26, 65, 27, 92},
        {28, 65, 27, 65},
        {27, 65, 29, 92},
        {25, 65, 30, 65},
        {26, 65, 28, 92},
        {24, 65, 32, 65},
        {30, 65, 31, 92},
        {31, 65, 34, 65},
        {32, 65, 33, 92},
        {33, 65, 36, 65},
        {35, 65, 31, 92},
        {38, 65, 35, 92},
        {37, 65, 33, 92},
        {40, 65, 37, 92},
        {38, 65, 39, 92},
        {42, 65, 39, 65},
        {40, 65, 41, 92},
        {41, 65, 44, 64},
        {42, 65, 43, 92},
        {46, 65, 43, 65},
        {44, 65, 45, 92},
        {45, 65, 48, 65},
        {48, 65, 49, 92},
        {46, 65, 47, 92},
        {47, 65, 51, 65},
        {49, 65, 50, 65}
};

void set_motion(int new_motion);
void setup();
int min_received_dist(int tab_dist[], int nb_bot);
void move_to_light();
void orbit_normal();
void orbit_tooclose();
void orbit_forward();
void orbit_stop();
void wait_togo();

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

    last_state_update = kilo_ticks;

    for (int i = 0; i < 3; ++i) {
        states[i] = ORBIT_STOP;
    }
    for (int i = 3; i < 12; ++i) {
        states[i] = MOVE_LIGHT;
    }
    for (int i = 12; i < NB_BOT; ++i) {
        states[i] = WAIT_TOGO;
    }

    for (int i = 0; i < NB_BOT; ++i) {
        range_dist[i] = 200;
    }
}

int min_received_dist(int tab_dist[], int nb_bot) {
    int return_min = 200;
    for (int i = 0; i < nb_bot; ++i) {
        if (tab_dist[i] < return_min) {
            return_min = tab_dist[i];
        }
    }
    return return_min;
}


/**
 * Allows a certain number of robots to wait before moving towards the shape.
 * This helps prevent congestion.
 */
void wait_togo(){
    set_motion(STOP);
    if (kilo_ticks > 9000 && kilo_uid >= 12 && kilo_uid < 22) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 30000 && kilo_uid >= 22 && kilo_uid < 30) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 50000 && kilo_uid >= 30 && kilo_uid < 35) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 65000 && kilo_uid >= 35 && kilo_uid < 40) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 80000 && kilo_uid >= 40 && kilo_uid < 45) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 90000 && kilo_uid >= 45 && kilo_uid < 49) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 100000 && kilo_uid >= 49 && kilo_uid < NB_BOT) {
        states[kilo_uid] = MOVE_LIGHT;
    }
}

/**
 * Function to resume the examples provided with the 'kilobot' plugin created by Vito Trianni and Carlo Pinciroli.
 * Addressed only to robots not present in the shape.
 * Measures the amount of perceived light and directs the robot towards the light source positioned above
 *     the origin of the shape. When a robot approaches the desired distance, it changes its state.
 */
void move_to_light(){
    int number_of_samples = 0;
    int sum = 0;

    int min = min_received_dist(range_dist, NB_BOT);

    if (states[kilo_uid] != ORBIT_STOP) {
        if (min <= DESIRED_DISTANCE ) {
            states[kilo_uid] = ORBIT_NORMAL;
        } else {
            while (number_of_samples < 300) {
                int sample = get_ambientlight();
                // -1 indicates a failed sample, which should be discarded.
                if (sample != -1) {
                    sum = sum + sample;
                    number_of_samples = number_of_samples + 1;
                }
            }

            // Compute the average.
            current_light = sum / number_of_samples;
//            printf("current light = %d\n", current_light);
            if (current_light < THRESH_LO) {
                set_motion(RIGHT);
//                printf("current_light < LO -> right\n");
            } else if (current_light > THRESH_HI) {
                set_motion(LEFT);
//                printf("current_light > HI -> left\n");
            } else {
                set_motion(LEFT);
            }

        }
    }
}

/**
 * The robot moves by oscillating from left to right along a desired orbit.
 * If the robot approaches a stationary robot, it changes its state - ORBIT_TO0CLOSE.
 * If the robot reaches the next location and it is free within the shape,
 *     it stops and changes its state - ORBIT_STOP.
 */
void orbit_normal() {

    int min = min_received_dist(range_dist, NB_BOT);

    if (min < TOOCLOSE_DISTANCE) {
        states[kilo_uid] = ORBIT_TOOCLOSE;
    } else {
        if (min < DESIRED_DISTANCE) {
            set_motion(LEFT);
        } else {
            set_motion(RIGHT);
        }
    }

    if (range_dist[forme[new_id[kilo_uid]][0]] >= (forme[new_id[kilo_uid]][1] - THRESHOLD) &&
        range_dist[forme[new_id[kilo_uid]][0]] <= (forme[new_id[kilo_uid]][1] + THRESHOLD) &&
        range_dist[forme[new_id[kilo_uid]][2]] >= (forme[new_id[kilo_uid]][3] - THRESHOLD) &&
        range_dist[forme[new_id[kilo_uid]][2]] <= (forme[new_id[kilo_uid]][3] + THRESHOLD)) {

        states[kilo_uid] = ORBIT_STOP;
    } else {
        // Its identifier is reset with an absurd value to avoid confusion.
        new_id[kilo_uid] = -1;
    }

}

/**
 * If the robot moves away from the nearest stationary robot, it changes its state - ORBIT_NORMAL.
 * Otherwise, it moves to the left to create distance.
 */
void orbit_tooclose() {

    int min = min_received_dist(range_dist, NB_BOT);

    if (min >= DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else if (range_dist[forme[new_id[kilo_uid]][0]] >= (forme[new_id[kilo_uid]][1] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][0]] <= (forme[new_id[kilo_uid]][1] + THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] >= (forme[new_id[kilo_uid]][3] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] <= (forme[new_id[kilo_uid]][3] + THRESHOLD)) {

        states[kilo_uid] = ORBIT_STOP;
    } else {
        set_motion(LEFT);
    }
}

/**
 * If the robot approaches a stationary robot at a distance less than desired,
 * it enters the 'ORBIT_NORMAL' state. Otherwise, it moves straight ahead.
 */
void orbit_forward() {

    int min = min_received_dist(range_dist, NB_BOT);
    if (received_state == ORBIT_STOP && min < DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else {
        set_motion(FORWARD);
    }
}

/**
 * The robot is within the shape, it stops moving and transmits its message.
 */
void orbit_stop() {
    set_motion(STOP);

    //If a robot stops within the shape, it considers itself present in the shape
    //     and updates the number of robots comprising the shape
    if (bot_in_form[kilo_uid] == 0) {
        bot_in_form[kilo_uid] = 1;
        nb_bot_form++;
    }

    // A robot within the shape transmits its message.
    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_form;
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);

}

void loop() {

    if (kilo_uid >2 && kilo_uid < 12 && received_one_mess == 0) {
        move_to_light();
    }
    //printf("%d : il y a %d robots dasn la forme\n", kilo_uid,nb_bot_form);
    if (message_sent == 1) {
        message_sent = 0;

        set_color(RGB(0, 1, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
    }

    if (new_message == 1) {
        new_message = 0;

        switch (states[kilo_uid]) {
            case ORBIT_NORMAL:
                orbit_normal();
                break;
            case ORBIT_TOOCLOSE:
                orbit_tooclose();
                break;
            case ORBIT_FORWARD:
                orbit_forward();
                break;
            case ORBIT_STOP:
                orbit_stop();
                break;
            case MOVE_LIGHT:
                move_to_light();
                break;
            case WAIT_TOGO:
                wait_togo();
                break;
            default:
                break;
        }
    } else if (states[kilo_uid] != STOP && states[kilo_uid] != WAIT_TOGO) {
        move_to_light();
    } else {
        wait_togo();
    }
}

void message_rx(message_t *m, distance_measurement_t *d) {

    new_message = 1;
    received_one_mess = 1;
    set_color(RGB(1, 0, 0));
    speaker_id = (*m).data[0];
    received_state = (*m).data[1];
    received_newID = (*m).data[2];
    if (nb_bot_form < (*m).data[3]) {
        nb_bot_form = (*m).data[3];
    }
    bot_in_form[speaker_id] = (*m).data[4]; //reçoit un booleen


    if (received_state == ORBIT_STOP) {
        range_dist[received_newID] = estimate_distance(d);
        // If its identifier is equal to an absurd value, then it is updated.
        if (new_id[kilo_uid] == -1) {
            new_id[kilo_uid] = nb_bot_form;
        }
    }
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


