
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

static const int NB_BOT = 36;
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
    COLLISION,
    WAIT_COLLISION,
    BROKEN_DOWN,
} orbit_state_t;

message_t message;
int message_sent = 0;
int new_message = 0;
int current_motion = STOP;
int speaker_id;
uint32_t last_ticks_update;
int new_id[NB_BOT] = {[0 ... 11] = -1};
int range_dist[NB_BOT] = {[0 ... 11] = 200};
int received_state;
int received_newID;
int current_light = 0, received_one_mess = 0;
uint32_t last_state_update;

int nb_bot_form = 3;

int bot_in_form[NB_BOT] = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


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
        {1, 65, 3, 92},
        {8, 65, 3, 65},
        {9, 65, 5, 65},
        {10, 65, 7, 65},
        {8, 65, 9, 92},
        {12, 65, 9, 65},
        {13, 65, 10, 65},
        {14, 65, 11, 65},
        {12, 65, 13, 92},
        {16, 65, 13, 65},
        {17, 65, 14, 65},
        {18, 65, 15, 65},
        {19, 65, 15, 92},
        {20, 65, 15, 65},
        {21, 65, 11, 65},
        {7, 65, 22, 65},
        {23, 65, 6, 65},
        {16, 65, 17, 92},
        {25, 65, 17, 65},
        {26, 65, 18, 65},
        {27, 65, 19, 65},
        {28, 65, 20, 65},
        {29, 65, 20, 92},
        {30, 65, 20, 65},
        {31, 65, 21, 65},
        {32, 65, 22, 65},
        {33, 65, 23, 65},
        {24, 65, 34, 65}
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
void collision(int fstate);
void wait_collision();
void broken_down();

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
        states[i] = WAIT_TOGO;
    }
    for (int i = 3; i < 5; ++i) {
        states[i] = MOVE_LIGHT;
    }
    for (int i = 5; i < NB_BOT; ++i) {
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

    //printf("%d wait_to_go\n", kilo_uid);

    set_motion(STOP);
    if (kilo_ticks > 9000 && kilo_uid >= 5 && kilo_uid < 8) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 18000 && kilo_uid >= 8 && kilo_uid < 11) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 27000 && kilo_uid >= 11 && kilo_uid < 14) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 36000 && kilo_uid >= 14 && kilo_uid < 17) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 45000 && kilo_uid >= 17 && kilo_uid < 20) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 54000 && kilo_uid >= 20 && kilo_uid < 23) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 63000 && kilo_uid >= 23 && kilo_uid < 26) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 72000 && kilo_uid >= 26 && kilo_uid < 29) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 81000 && kilo_uid >= 29 && kilo_uid < 32) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 90000 && kilo_uid >= 32 && kilo_uid < 35) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 99000 && kilo_uid >= 35 && kilo_uid < 38) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 108000 && kilo_uid >= 38 && kilo_uid < 41) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 117000 && kilo_uid >= 41 && kilo_uid < 44) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 126000 && kilo_uid >= 44 && kilo_uid < 48) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 135000 && kilo_uid >= 48 && kilo_uid < 51) {
        states[kilo_uid] = MOVE_LIGHT;
    }
    if (kilo_ticks > 144000 && kilo_uid >= 51 && kilo_uid < NB_BOT) {
        states[kilo_uid] = MOVE_LIGHT;
    }
}

void broken_down(){
    states[kilo_uid] = BROKEN_DOWN;
}

void collision(int fstate){
    //printf("%d collision\n", kilo_uid);
    //printf("Collision : kilo_ticks = %d, last_ticks = %d\n", kilo_ticks, last_ticks_update);
    //printf("coll : %d change d'état -> WAIT_COLLISION\n", kilo_uid);
    states[kilo_uid] = WAIT_COLLISION;

}

void wait_collision(){

    //printf("%d wait_collison\n", kilo_uid);

    if (kilo_ticks < (last_ticks_update + 100)){
        //printf("%d arrête ses moteurs.  kt = %d, last_update = %d\n", kilo_uid, kilo_ticks, last_ticks_update);
        set_motion(STOP);
        states[kilo_uid] = WAIT_COLLISION;
    } else {
        last_ticks_update = kilo_ticks;
        //printf("%d retourne en movelight. kt = %d, last_update = %d\n ", kilo_uid, kilo_ticks, last_ticks_update);
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

    //printf("%d movelight\n", kilo_uid);

    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_form;
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);

    if (received_state == MOVE_LIGHT){
        if (kilo_uid > speaker_id){
            //printf("%d change d'état -> COLLISION\n",kilo_uid);
            states[kilo_uid] = COLLISION;
        }
    }

    int min = min_received_dist(range_dist, NB_BOT);

    if (states[kilo_uid] != ORBIT_STOP) {
        if (min < (DESIRED_DISTANCE - 1) ) {
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
//            //printf("current light = %d\n", current_light);
            if (current_light < THRESH_LO) {
                set_motion(RIGHT);
//                //printf("current_light < LO -> right\n");
            } else if (current_light > THRESH_HI) {
                set_motion(LEFT);
//                //printf("current_light > HI -> left\n");
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

    //printf("%d normal\n", kilo_uid);

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
        printf("%d change d'état : normal -> Stop", kilo_uid);
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

    //printf("%d tooclose\n", kilo_uid);

    if (min >= DESIRED_DISTANCE) {
        states[kilo_uid] = ORBIT_NORMAL;
    } else if (range_dist[forme[new_id[kilo_uid]][0]] >= (forme[new_id[kilo_uid]][1] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][0]] <= (forme[new_id[kilo_uid]][1] + THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] >= (forme[new_id[kilo_uid]][3] - THRESHOLD) &&
               range_dist[forme[new_id[kilo_uid]][2]] <= (forme[new_id[kilo_uid]][3] + THRESHOLD)) {
        printf("%d change d'état : tooclose -> Stop", kilo_uid);
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

    //printf("%d forward\n", kilo_uid);

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

    //printf("%d stop\n", kilo_uid);

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

    if (states[kilo_uid] == MOVE_LIGHT){
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
            case COLLISION:
                collision(states[kilo_uid]);
                break;
            case WAIT_COLLISION:
                wait_collision();
                break;
            case BROKEN_DOWN:
                broken_down();
                break;
            default:
                break;
        }
    } else if (states[kilo_uid] != ORBIT_STOP && states[kilo_uid] != WAIT_TOGO ) {
        move_to_light();
    } else if (states[kilo_uid] != ORBIT_STOP){
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


