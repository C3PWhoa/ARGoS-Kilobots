//
// Created by frederic turchet on 07/05/2023.
//
#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define THRESHOLD 7
#define STOP_DISTANCE 55

static const int NB_BOT = 7;
static const uint8_t TOOCLOSE_DISTANCE = 40; // 35 mm
static const uint8_t DESIRED_DISTANCE = 65; // 70 mm
static const uint8_t TOOFAR_DISTANCE = 99; // 99 mm

typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
    RANDOM_WALK,
} orbit_state_t;

message_t message;
int message_sent = 0;
int new_message = 0;
int current_motion = STOP;
int speaker_id;
int last_ticks_update;
int new_id[NB_BOT] = {[0 ... NB_BOT - 1] = 200};
int range_dist[NB_BOT] = {[0 ... NB_BOT - 1] = 200};
int received_state;
int received_newID;

int nb_bot_form = 3;
int bot_in_form[NB_BOT] = {0,0,0,0,0,0,0};

orbit_state_t state = ORBIT_FORWARD;
orbit_state_t states[NB_BOT];

//[new_id]:{n1, d1, n2, d2, new_id} where 0 < ni < N_BOT and 'd' equal's distance between kilo_uid and ni
int forme[NB_BOT][5] = {
        {1, 65, 2, 65, 0},
        {0, 65, 2, 92, 1},
        {0, 65, 1, 92, 2},
        {1, 65, 2, 65, 3},
        {2, 65, 3, 92, 4},
        {3, 65, 4, 65, 4},
        {3, 65, 1, 92, 4}
};


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

    for (int i = 3; i < NB_BOT; ++i) {
        states[i] = ORBIT_FORWARD;
    }

    for (int i = 0; i < NB_BOT; ++i) {
        new_id[i] = kilo_uid;
    }

    for (int i = 0; i < NB_BOT; ++i) {
        range_dist[i] = 200;
    }



    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.crc = message_crc(&message);

    printf("__________________SETUP________________________\n");
    printf("kilo_uid = %d\n", kilo_uid);
    printf("states[%d] = %d\n", kilo_uid, states[kilo_uid]);
    printf("new_id[%d] = %d\n", kilo_uid, new_id[0]);
    printf("send message : kilo_uid: %d, states: %d, new_id: %d\n", kilo_uid, states[kilo_uid], new_id[kilo_uid]);
    printf("****>>> nb de robot dans la forme : %d\n", nb_bot_form);
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

int nb_bot_in_form(int tabStates[], int nb_bot){
    int nbBot = 0;
    for (int i = 0; i < nb_bot; ++i) {
        if (tabStates[i] == ORBIT_STOP){
            nbBot++;
        }
    }
    return nbBot;
}

void orbit_normal() {
    printf("_________________NORMAL____________________\n");
    printf("-> %d est dans NORMAL\n", kilo_uid);

    int min = min_received_dist(range_dist, NB_BOT);
    printf("min = %d\n", min);

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
        printf("***********NORMAL****************\n");
        states[kilo_uid] = ORBIT_STOP;
    }

}

void orbit_tooclose() {
    printf("________________TOOCLOSE_______________\n");
    printf("-> %d est dans TOOCLOSE\n", kilo_uid);

    int min = min_received_dist(range_dist, NB_BOT);
    printf("min tooclose= %d\n", min);

    if (min >= DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else if (range_dist[forme[new_id[kilo_uid]][0]] >= (forme[new_id[kilo_uid]][1] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][0]] <= (forme[new_id[kilo_uid]][1] + THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] >= (forme[new_id[kilo_uid]][3] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] <= (forme[new_id[kilo_uid]][3] + THRESHOLD)) {
        printf("***********TOOCLOSE****************\n");
        states[kilo_uid] = ORBIT_STOP;
    } else {
        set_motion(FORWARD);
    }
}

void orbit_forward() {
    printf("_______________FORWARD___________________\n");
    printf("-> %d est dans FORWARD\n", kilo_uid);

    int min = min_received_dist(range_dist, NB_BOT);
    if (received_state == ORBIT_STOP && min < DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else {
        if (kilo_ticks > last_ticks_update + 1000) {
            last_ticks_update = kilo_ticks;

            set_motion(FORWARD);
        }
    }
}

void orbit_stop() {
    printf("__________STOP_____________________\n");
    printf("-> %d est dans STOP\n", kilo_uid);
    set_motion(STOP);

    int nbBotInForm = 0;

    printf("bot_in_form[%d] = %d\n", kilo_uid-1, bot_in_form[kilo_uid]);

    printf("****>>> nb de robot dans la forme : %d\n", nb_bot_in_form(states, NB_BOT));

    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.crc = message_crc(&message);

}

void random_walk() {
    printf("__________RANDOM_____________________\n");
    printf("-> %d est dans RANDOM_WALK\n", kilo_uid);
    printf("received state = %d\n", received_state);
    int min = min_received_dist(range_dist, NB_BOT);

    // Si le robot reçoit un message d'un robot stationnaire alors il change d'état -> NORMAL
    // Sinon, il se déplace dans une direction aléatoire pendant 250 ticks
    if (received_state == ORBIT_STOP && min < DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else {
        if (kilo_ticks > last_ticks_update + 1000) {
            last_ticks_update = kilo_ticks;

            int rand_number = rand_hard();
            int rand_direction = (rand_number % 4);
            if (rand_direction == 0 || rand_direction == 1) {
                set_motion(FORWARD);
                printf("random forward\n");
            } else if (rand_direction == 2) {
                set_motion(RIGHT);
                printf("random right\n");
            } else {
                set_motion(LEFT);
                printf("random left\n");
            }
        }
    }



//        message.type = NORMAL;
//        message.data[0] = kilo_uid;
//        message.data[1] = states[kilo_uid];
//        message.data[2] = new_id[kilo_uid];
//        message.crc = message_crc(&message);

}

void loop() {

    printf("%d est dans la loop et son état = %d\n", kilo_uid, states[kilo_uid]);
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
            case RANDOM_WALK:
                random_walk();
                break;
            default:
                break;
        }
    } else {
        orbit_forward();
    }
}

void message_rx(message_t *m, distance_measurement_t *d) {
    printf("______________message_rx_________________\n");

    new_message = 1;
    set_color(RGB(1, 0, 0));
    speaker_id = (*m).data[0];
    received_state = (*m).data[1];
    received_newID = (*m).data[2];
    if (received_state == ORBIT_STOP) {
        range_dist[received_newID] = estimate_distance(d);
        //if (kilo_uid == 4) {
        printf("%d a reçu un message de %d. state=%d, new_id= %d, range_dist[%d] = %d.\n", kilo_uid, speaker_id,
               received_state, received_newID, speaker_id, range_dist[received_newID]);
        printf("son nouvel id est %d\n", new_id[kilo_uid]);
        //}

    }
}


message_t *message_tx() {
    printf("%d a envoyé un message\n", kilo_uid);
    printf("son état: %d\n", states[kilo_uid]);
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
