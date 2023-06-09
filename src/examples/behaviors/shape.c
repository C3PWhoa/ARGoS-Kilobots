
#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

#define THRESHOLD 10
#define STOP_DISTANCE 55

// Constants for light following.
#define THRESH_LO 300
#define THRESH_HI 600

static const int NB_BOT = 7;
static const uint8_t TOOCLOSE_DISTANCE = 35; // 35 mm
static const uint8_t DESIRED_DISTANCE = 40; // 70 mm
static const uint8_t TOOFAR_DISTANCE = 80; // 99 mm

typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
    RANDOM_WALK,
    MOVE_LIGHT,
} orbit_state_t;

message_t message;
int message_sent = 0;
int new_message = 0;
int current_motion = STOP;
int current_light = 0;
int speaker_id;
int last_ticks_update;
int new_id[NB_BOT] = {[0 ... NB_BOT - 1] = -1};
int range_dist[NB_BOT] = {[0 ... NB_BOT - 1] = 200};
int received_state;
int received_newID;
int received_one_mess = 0;
int nbTicks = 0;
int i = 0;

int nb_bot_form = 3;
int bot_in_form[NB_BOT] = {1,1,1,0,0,0,0};

orbit_state_t state = ORBIT_FORWARD;
orbit_state_t states[NB_BOT];

//[new_id]:{n1, d1, n2, d2, new_id} where 0 < ni < N_BOT and 'di' equal's distance between this and ni
int forme[NB_BOT][5] = {
        {1, 70, 2, 70, 0},
        {0, 70, 2, 99, 1},
        {0, 70, 2, 70, 2},
        {1, 70, 2, 70, 3},
        {2, 70, 3, 99, 4},
        {3, 70, 4, 70, 5},
        {3, 70, 1, 99, 6}
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
void random_walk();
void waiting();

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
    for (int i = 3; i < NB_BOT; ++i) {
        states[i] = MOVE_LIGHT;
    }

    for (int i = 0; i < 3; ++i) {
        new_id[i] = kilo_uid;
    }

    //init range_dist pour etre sûr d'avoir un minimum
    for (int i = 0; i < NB_BOT; ++i) {
        range_dist[i] = 200;
    }



    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_form;
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);

//    printf("(SETUP) kilo_uid = %d\n", kilo_uid);
//    printf("(SETUP) states[%d] = %d\n", kilo_uid, states[kilo_uid]);
//    printf("(SETUP) new_id[%d] = %d\n", kilo_uid, new_id[0]);
//    printf("(SETUP) send message : kilo_uid: %d, states: %d, new_id: %d\n", kilo_uid, states[kilo_uid], new_id[kilo_uid]);
//    printf("(SETUP) ****>>> nb de robot dans la forme : %d\n", nb_bot_form);
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

int nb_bot_in_form(){
    int nbBot = 0;
    for (int i = 0; i < NB_BOT; ++i) {
        if (bot_in_form[i] == 1){
            //printf("(nb_bot_in_form())!! bot_in_form[%d] = %d\n",i , bot_in_form[i]);
            nbBot++;
        }
    }
    //printf("(nb_bot_in_form()) %d : nbBot in form = %d\n", kilo_uid, nbBot);
    return nbBot;
}
void move_to_light(){
//    printf("kb%d est dans move to light\n", kilo_uid);
    int number_of_samples = 0;
    int sum = 0;

    int min = min_received_dist(range_dist, NB_BOT);

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
    //printf("(NORMAL) -> %d est dans NORMAL\n", kilo_uid);

    int min = min_received_dist(range_dist, NB_BOT);
    //printf("(NORMAL) min = %d\n", min);

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
//        printf("%d Stop dans NORMAL\n", kilo_uid);
        states[kilo_uid] = ORBIT_STOP;
    } else {
        new_id[kilo_uid] = -1;
    }

}

void orbit_tooclose() {
    // printf("(TOOCLOSE) -> %d est dans TOOCLOSE\n", kilo_uid);

    int min = min_received_dist(range_dist, NB_BOT);

    if (min >= DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else if (range_dist[forme[new_id[kilo_uid]][0]] >= (forme[new_id[kilo_uid]][1] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][0]] <= (forme[new_id[kilo_uid]][1] + THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] >= (forme[new_id[kilo_uid]][3] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] <= (forme[new_id[kilo_uid]][3] + THRESHOLD)) {
//        printf("%d Stop dans TOOCLOSE\n", kilo_uid);
        states[kilo_uid] = ORBIT_STOP;
    }
//    else if (min <= 57 && min >= 40){
//        set_motion(LEFT);
//    }
    else {
        if(min < TOOCLOSE_DISTANCE){
            set_motion(LEFT);
        } else {
            set_motion(FORWARD);
        }
    }
}

void orbit_forward() {
    int min = min_received_dist(range_dist, NB_BOT);
    if (states[kilo_uid] != ORBIT_STOP) {
        if (received_state == ORBIT_STOP && (min < TOOFAR_DISTANCE && min > DESIRED_DISTANCE) ) {
            set_motion(LEFT);
        } else if (received_state == ORBIT_STOP && min < DESIRED_DISTANCE){
            states[kilo_uid] = ORBIT_NORMAL;
        } else {
            set_motion(FORWARD);
        }
    }
}

void orbit_stop() {
    //printf("(STOP) -> %d est dans STOP\n", kilo_uid);
    set_motion(STOP);

    if (bot_in_form[kilo_uid] == 0){
//        printf("(STOP) je change dans bot_in_form\n");
        bot_in_form[kilo_uid] = 1;
    }
    // printf("(STOP) %d : il y a %d robot dans la forme\n", kilo_uid, nb_bot_in_form());

    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_in_form();
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);

}

void random_walk() {
//    printf("__________RANDOM_____________________\n");
//    printf("-> %d est dans RANDOM_WALK\n", kilo_uid);
    int min = min_received_dist(range_dist, NB_BOT);

    // Si le robot reçoit un message d'un robot stationnaire alors il change d'état -> NORMAL
    // Sinon, il se déplace dans une direction aléatoire pendant 250 ticks
    if (states[kilo_uid] != ORBIT_STOP) {
        if (received_state == ORBIT_STOP && min < DESIRED_DISTANCE) {
            states[kilo_uid] = ORBIT_NORMAL;
        } else {
            if (kilo_ticks > last_ticks_update + 50) {
                last_ticks_update = kilo_ticks;

                int rand_number = rand_hard();
//                printf("rand_hard : %\n", rand_number);
                int rand_direction = (rand_number % 4);
//                printf("rand_direction : %d\n", rand_direction);
                if (rand_direction == 0 || rand_direction == 1) {
                    set_motion(FORWARD);
                } else if (rand_direction == 2) {
                    set_motion(RIGHT);
                } else {
                    set_motion(LEFT);
                }
            }
        }
    }
}

void loop() {

    if (kilo_uid >2 && received_one_mess == 0) {
        move_to_light();
    }



    //printf("%d est dans la loop et son état = %d\n", kilo_uid, states[kilo_uid]);
    if (message_sent == 1) {
        message_sent = 0;

        set_color(RGB(0, 1, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
    }

    if (new_message == 1) {
        new_message = 0;

        switch (states[kilo_uid]) {
            case MOVE_LIGHT:
                move_to_light();
                break;
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
    } else if (states[kilo_uid] != STOP){
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
    //if (nb_bot_form < (*m).data[3]){
    nb_bot_form = (*m).data[3];
    //}
    bot_in_form[speaker_id] = (*m).data[4]; //reçoit un booleen


    if (received_state == ORBIT_STOP) {
        range_dist[received_newID] = estimate_distance(d);
        if (new_id[kilo_uid] == -1) {
            new_id[kilo_uid] = nb_bot_form;
            //printf("(condition RX) %d pense qu'il y a %d robot dans la forme\n", kilo_uid, (*m).data[3]);
        }

//        printf("(RX) %d pense qu'il y a %d robot dans la forme\n", kilo_uid, (*m).data[3]);
//        printf("(RX) %d : new id = %d\n", kilo_uid, new_id[kilo_uid]);
        printf("(RX) %d : a reçu un message de % d (new_id : %d), distance = %d\n", kilo_uid, speaker_id, received_newID, range_dist[received_newID]);
        if ((*m).data[3] != NB_BOT){
            nbTicks = kilo_ticks;
        } else {
            i++;
            printf("[%d] - l'expérience a duré %d ticks\n", i, nbTicks);
            printf("(RX) %d : a reçu un message de % d (new_id : %d), distance = %d\n", kilo_uid, speaker_id,
                   received_newID, range_dist[received_newID]);
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