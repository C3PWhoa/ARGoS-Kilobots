
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
#define THRESHOLD 7

static const int NB_BOT = 12;
static const uint8_t TOOCLOSE_DISTANCE = 45;
static const uint8_t DESIRED_DISTANCE = 65;
static const uint8_t TOOFAR_DISTANCE = 80;

typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
    MOVE_LIGHT,
} orbit_state_t;

message_t message;
int message_sent = 0;
int new_message = 0;
int current_motion = STOP;
int speaker_id;
int last_ticks_update;
int new_id[12] = {[0 ... 11] = -1};
int range_dist[12] = {[0 ... 11] = 200};
int received_state;
int received_newID;
int current_light = 0, received_one_mess = 0;

int nb_bot_form = 3;
int bot_in_form[12] = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

orbit_state_t state = ORBIT_FORWARD;
orbit_state_t states[12];



//[new_id]:{n1, d1, n2, d2} where 0 < ni < N_BOT and 'di' equal's distance between this and ni
// ---- 6  7 ----
// - 1  3  5  10 -
// - 0  2  4  11 -
// ---- 8  9 ----
int forme[12][4] = {
        {1, 65, 2, 65},
        {0, 65, 2, 92},
        {0, 65, 1, 92},
        {1, 65, 2, 65},
        {2, 65, 3, 92},
        {3, 65, 4, 65},
        {3, 65, 5, 92},
        {6, 65, 5, 65},
        {5, 65, 4, 92},
        {8, 65, 4, 65},
        {4, 65, 9, 92},
        {2, 65, 10, 65}
};

void set_motion(int new_motion);
void setup();
int min_received_dist(int tab_dist[], int nb_bot);
int nb_bot_in_form();
void move_to_light();
void orbit_normal();
void orbit_tooclose();
void orbit_forward();
void orbit_stop();

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

    for (int i = 0; i < 3; ++i) {
        states[i] = ORBIT_STOP;
    }
    for (int i = 3; i < 12; ++i) {
        states[i] = MOVE_LIGHT;
    }

    for (int i = 0; i < 12; ++i) {
        range_dist[i] = 200;
    }


    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_form;
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);
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

int nb_bot_in_form() {
    int nbBot = 0;
    for (int i = 0; i < 12; ++i) {
        if (bot_in_form[i] == 1) {
            nbBot++;
        }
    }
    return nbBot;
}

void move_to_light(){
    int number_of_samples = 0;
    int sum = 0;

    int min = min_received_dist(range_dist, 12);

    if (states[kilo_uid] != ORBIT_STOP) {
        if (min <= TOOFAR_DISTANCE + 5) {
            states[kilo_uid] = ORBIT_FORWARD;
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

void orbit_normal() {

    int min = min_received_dist(range_dist, 12);

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

void orbit_tooclose() {

    int min = min_received_dist(range_dist, 12);

    if (min >= DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else if (range_dist[forme[new_id[kilo_uid]][0]] >= (forme[new_id[kilo_uid]][1] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][0]] <= (forme[new_id[kilo_uid]][1] + THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] >= (forme[new_id[kilo_uid]][3] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] <= (forme[new_id[kilo_uid]][3] + THRESHOLD)) {

        states[kilo_uid] = ORBIT_STOP;
    } else {
        set_motion(FORWARD);
    }
}

void orbit_forward() {

    int min = min_received_dist(range_dist, 12);
    if (received_state == ORBIT_STOP && min < DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else {
        set_motion(FORWARD);
    }
}

void orbit_stop() {
    set_motion(STOP);

    if (bot_in_form[kilo_uid] == 0) {
        bot_in_form[kilo_uid] = 1;
    }

    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_in_form();
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);

}

void loop() {

    if (kilo_uid >2 && received_one_mess == 0) {
        move_to_light();
    }

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
            default:
                break;
        }
    } else if (states[kilo_uid] != STOP) {
        move_to_light();
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
