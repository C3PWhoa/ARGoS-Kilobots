#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>

message_t message;
int message_sent = 0;
int new_message = 0;
int speaker_id;
int new_id[3];
int nb_bot_form = 3;
int received_state;
int received_newID;
static const int NB_BOT = 12;
int bot_in_form[12] = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
} orbit_state_t;

orbit_state_t states[3];

void setup() {

    new_id[0] = 0;
    new_id[1] = 1;
    new_id[2] = 2;

    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = ORBIT_STOP;
    message.data[2] = new_id[kilo_uid];
    message.data[3] = nb_bot_form;
    message.data[4] = bot_in_form[kilo_uid];
    message.crc = message_crc(&message);
}


void loop() {
    // Blink LED magenta whenever a message is sent.
    printf("%d : il y a %d robots dasn la forme\n", kilo_uid,nb_bot_form);
    if (message_sent == 1) {
        message_sent = 0;

        set_color(RGB(1, 0, 1));
        delay(100);
        set_color(RGB(0, 0, 0));

        message.type = NORMAL;
        message.data[0] = kilo_uid;
        message.data[1] = ORBIT_STOP;
        message.data[2] = new_id[kilo_uid];
        message.data[3] = nb_bot_form;
        message.data[4] = bot_in_form[kilo_uid];
        message.crc = message_crc(&message);
    }

    if (new_message == 1) {
        new_message = 0;
    }
}

void message_rx(message_t *m, distance_measurement_t *d) {
    new_message = 1;
    speaker_id = (*m).data[0];
    received_state = (*m).data[1];
    received_newID = (*m).data[2];
    if (nb_bot_form < (*m).data[3]){
        nb_bot_form = (*m).data[3];
    }
    message.data[4] = bot_in_form[kilo_uid];
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

