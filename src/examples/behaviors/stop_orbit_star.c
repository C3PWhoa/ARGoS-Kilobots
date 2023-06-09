//
// Created by frederic turchet on 09/06/2023.
//
#include <kilolib.h>
#include <stdio.h>
#include <stdlib.h>

static const int NB_BOT = 4;

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

message_t message;
// Flag to keep track of message transmission.
int message_sent = 0;
int new_message = 0;
int speaker_id, received_state;
int range_dist[NB_BOT];

void setup()
{
    for (int i = 0; i < 3; ++i) {
        states[i] = ORBIT_STOP;
    }

    // Initialize message:
    // The type is always NORMAL.
    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.crc = message_crc(&message);
}

void loop()
{
    // Blink LED magenta whenever a message is sent.
    if (message_sent == 1)
    {
        // Reset flag so LED is only blinked once per message.
        message_sent = 0;

        set_color(RGB(1, 0, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
    }

    if (new_message == 1){
        new_message = 0;

        set_color(RGB(0,1,0));
    }
}

void message_rx(message_t *m, distance_measurement_t *d){
    new_message = 1;
    speaker_id = (*m).data[0];
    received_state = (*m).data[1];
    range_dist[speaker_id] = estimate_distance(d);
}

message_t *message_tx()
{
    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.data[1] = states[kilo_uid];
    message.crc = message_crc(&message);

    return &message;
}

void message_tx_success()
{
    // Set flag on message transmission.
    message_sent = 1;
}

int main()
{
    kilo_init();
    // Register the message_tx callback function.
    kilo_message_tx = message_tx;
    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;

    kilo_message_rx = message_rx;
    kilo_start(setup, loop);

    return 0;
}
