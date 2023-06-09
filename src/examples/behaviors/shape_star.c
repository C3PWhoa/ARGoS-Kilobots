//
// Created by frederic turchet on 13/05/2023.
//
#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>

static const int NB_BOT = 7;

message_t message;
int message_sent = 0;
int new_message = 0;
int speaker_id;
int new_id[NB_BOT];
int bot_in_form[NB_BOT];
int range_dist[NB_BOT] = {[0 ... NB_BOT - 1] = 200};

int nb_bot_form = 3;
int received_state;
int received_newID;

typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
    RANDOM_WALK,
    MOVE_LIGHT,
    WAIT,
} orbit_state_t;

orbit_state_t states[NB_BOT];

orbit_state_t state = ORBIT_STOP;

int nb_bot_in_form(){
    int nbBot = 0;
    for (int i = 0; i < NB_BOT; ++i) {
        if (bot_in_form[i] == 1){
            //printf("(nb_bot_in_form())!! bot_in_form[%d] = %d\n",i , bot_in_form[i]);
            nbBot++;
        }
    }
    printf("(nb_bot_in_form()) %d : nbBot in form = %d\n", kilo_uid, nbBot);
    return nbBot;
}

void setup() {

    for (int i = 0; i < 3; ++i) {
        states[i] = ORBIT_STOP;
    }
    for (int i = 3; i < NB_BOT; ++i) {
        states[i] = MOVE_LIGHT;
    }

    //INTEGER : Init new_id[0...2] == kilo_uid && new_id[3...NB_BOT] == -1
    for (int i = 0; i < 3; ++i) {
        new_id[i] = kilo_uid;
    }
    for (int j = 3; j < NB_BOT; ++j) {
        new_id[j] = -1;
    }

    //BOOLEAN : Init bot_in_form[0...2] == 1 && bot_in_form[3...NB_BOT] == 0
    for (int i = 0; i < 3; ++i) {
        bot_in_form[i] = 1;
    }
    for (int i = 3; i < NB_BOT; ++i) {
        bot_in_form[i] = 0;
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
    // Blink LED blue whenever a message is sent.
    if (message_sent == 1) {
        message_sent = 0;



        set_color(RGB(1, 0, 0));
        delay(50);
        set_color(RGB(0, 0, 0));
    }

    // Blink LED red whenever a message is received.
    if (new_message == 1) {
        new_message = 0;

        set_color(RGB(1, 0, 0));
        delay(50);
        set_color(RGB(0, 0, 0));

    }
}

void message_rx(message_t *m, distance_measurement_t *d) {
    new_message = 1;
    speaker_id = (*m).data[0];
    received_state = (*m).data[1];
    received_newID = (*m).data[2];
    nb_bot_form = nb_bot_in_form();
    bot_in_form[speaker_id] = (*m).data[4];

    if (received_state == ORBIT_STOP) {
        range_dist[received_newID] = estimate_distance(d);
        if (new_id[kilo_uid] == -1) {
            new_id[kilo_uid] = nb_bot_form;
            //printf("(condition RX) %d pense qu'il y a %d robot dans la forme\n", kilo_uid, (*m).data[3]);
        }
        printf("(RX) %d : a reçu un message de % d (new_id : %d), distance = %d\n", kilo_uid, speaker_id, received_newID, range_dist[received_newID]);
    }
}

message_t *message_tx() {
    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_in_form();
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);

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