
#include <kilolib.h>
#include <stdlib.h>
#include <stdio.h>

// Constants for orbit control.
#define TOO_CLOSE_DISTANCE 40
#define DESIRED_DISTANCE 60

// Constants for motion handling function.
#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

int current_motion = STOP;
int distance;
int new_message = 0, message_sent = 0;
message_t message;

int speaker_uid;

// Function to handle motion.
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
    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.crc = message_crc(&message);
}

void loop() {
    // Blink LED magenta whenever a message is sent.
    if (message_sent == 1) {
        // Reset flag so LED is only blinked once per message.
        message_sent = 0;

        set_color(RGB(1, 0, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
        printf("Message sent\n");
    }
    // Print distances between two kilobots
    if (new_message == 1) {
        new_message = 0;

        printf("Distance entre kb%d et kb%d = %dmm\n ", kilo_uid, speaker_uid, distance);

    }
}

message_t *message_tx() {
    return &message;
}

void message_tx_success() {
    // Set flag on message transmission.
    message_sent = 1;
}

void message_rx(message_t *m, distance_measurement_t *d) {
    new_message = 1;
    distance = estimate_distance(d);
    speaker_uid = (*m).data[0];
}

int main() {
    kilo_init();
    // Register the message_tx callback function.
    kilo_message_tx = message_tx;
    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;
    // Register the message_rx callback function.
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);

    return 0;
}
