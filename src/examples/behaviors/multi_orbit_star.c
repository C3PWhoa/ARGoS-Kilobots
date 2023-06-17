//
// Created by frederic turchet on 16/04/2023.
//
#include <kilolib.h>

message_t message;
int message_sent = 0;
int new_id[3];
typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
    ORBIT_FORWARD,
    ORBIT_STOP,
    RANDOM_WALK,
} orbit_state_t;

orbit_state_t states[3];

void setup() {

    for (int i = 0; i < 3; ++i) {
        new_id[i] = kilo_uid;
    }

    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = ORBIT_STOP;
    message.data[2] = new_id[kilo_uid];
    message.crc = message_crc(&message);
}

void loop() {
    // Blink LED magenta whenever a message is sent.
    if (message_sent == 1) {
        message_sent = 0;

        set_color(RGB(1, 0, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
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
    kilo_start(setup, loop);

    return 0;
}
