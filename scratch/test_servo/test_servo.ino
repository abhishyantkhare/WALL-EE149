/*
 * 4 Servo Sweep
*/

#include <Servo.h>

// twelve servo objects can be created on most boards

const byte num_servos = 4;
Servo servo_array[num_servos];
int pos_array[num_servos] = {0, 0, 0, 0};
int start_pin = 2;

int curr_pos;
Servo curr_servo;

void setup() {
  for (int i = 0; i < num_servos; i+= 1) {
    servo_array[i].attach(start_pin + i);
  }
}

void loop() {  
  for (int i = 0; i < num_servos; i += 1) {
    curr_pos = pos_array[i];
    curr_servo = servo_array[i];

    for (curr_pos = 0; curr_pos <= 180; curr_pos += 1){
      curr_servo.write(curr_pos);
      delay(15);
    }
    for (curr_pos = 180; curr_pos >= 0; curr_pos -= 1){
      curr_servo.write(curr_pos);
      delay(15);
    }
  }
}

