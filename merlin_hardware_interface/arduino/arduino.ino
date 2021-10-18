#define WAIST_PULSE_PIN 7
#define WAIST_DIR_PIN 6
#define WAIST_BRAKE_PIN 8

#define SHOULDER_PULSE_PIN 5
#define SHOULDER_DIR_PIN 4
#define SHOULDER_BRAKE_PIN 7

#define ELBOW_PULSE_PIN 3
#define ELBOW_DIR_PIN 2
#define ELBOW_BRAKE_PIN 10

#define WRIST_AXIS_1_PULSE_PIN 14
#define WRIST_AXIS_1_DIR_PIN 6

#define WRIST_AXIS_2_PULSE_PIN 15
#define WRIST_AXIS_2_DIR_PIN 6

#define WRIST_AXIS_3_PULSE_PIN 15
#define WRIST_AXIS_3_DIR_PIN 6

#define ONE_DEGREE 0.0175

float waist_rps = 0.00065; // Radian Per Step

float shoulder_rps = 0.00065; // Radian Per Step

float elbow_rps = 0.00065; // Radian Per Step

union open_float
{
  char bytes[4];
  float value = 0;
};

open_float waist;
open_float shoulder;
open_float elbow;

open_float wrist_rotation;
open_float wrist_flex;
open_float hand_roll;

open_float waist_target;
open_float shoulder_target;
open_float elbow_target;

open_float wrist_rotation_target;
open_float wrist_flex_target;
open_float hand_roll_target;

void setup_pins()
{
  pinMode(WAIST_PULSE_PIN, OUTPUT);
  pinMode(WAIST_DIR_PIN, OUTPUT);
  pinMode(WAIST_BRAKE_PIN, OUTPUT);

  pinMode(SHOULDER_PULSE_PIN, OUTPUT);
  pinMode(SHOULDER_DIR_PIN, OUTPUT);
  pinMode(SHOULDER_BRAKE_PIN, OUTPUT);

  pinMode(ELBOW_PULSE_PIN, OUTPUT);
  pinMode(ELBOW_DIR_PIN, OUTPUT);
  pinMode(ELBOW_BRAKE_PIN, OUTPUT);

  pinMode(WRIST_AXIS_1_PULSE_PIN, OUTPUT);
  pinMode(WRIST_AXIS_1_DIR_PIN, OUTPUT);

  pinMode(WRIST_AXIS_2_PULSE_PIN, OUTPUT);
  pinMode(WRIST_AXIS_2_DIR_PIN, OUTPUT);

  pinMode(WRIST_AXIS_3_PULSE_PIN, OUTPUT);
  pinMode(WRIST_AXIS_3_DIR_PIN, OUTPUT);
}

void load_saved_state()
{
  waist.value = 0;
  shoulder.value = 0;
  elbow.value = 0;
  wrist_rotation.value = 0;
  wrist_flex.value = 0;
  hand_roll.value = 0;
}

void unlock_brakes(){

}

void lock_brakes(){

}

void setup_arm()
{
  setup_pins();
  load_saved_state();
  unlock_brakes();
}

void setup()
{
  Serial.begin(115200);
  setup_arm();
}

void update_arm()
{
  if (abs(waist.value - waist_target.value) > ONE_DEGREE / 2)
  {
    digitalWrite(WAIST_DIR_PIN, (waist.value < waist_target.value) ? LOW : HIGH);
    waist.value += (waist.value < waist_target.value) ? waist_rps : -waist_rps;
    digitalWrite(WAIST_PULSE_PIN, HIGH);
    delay(6);
    digitalWrite(WAIST_PULSE_PIN, LOW);
  }
  if (abs(shoulder.value - shoulder_target.value) > ONE_DEGREE / 2)
  {
    digitalWrite(SHOULDER_DIR_PIN, (shoulder.value < shoulder_target.value) ? HIGH : LOW);
    // Account for -1 Mechanical Reduction
    elbow.value += (shoulder.value < shoulder_target.value) ? shoulder_rps : -shoulder_rps;
    shoulder.value += (shoulder.value < shoulder_target.value) ? shoulder_rps : -shoulder_rps;
    digitalWrite(SHOULDER_PULSE_PIN, HIGH);
    delay(4);
    digitalWrite(SHOULDER_PULSE_PIN, LOW);
  }
  if (abs(elbow.value - elbow_target.value) > ONE_DEGREE / 2)
  {
    digitalWrite(ELBOW_DIR_PIN, (elbow.value < elbow_target.value) ? HIGH : LOW);
    elbow.value += (elbow.value < elbow_target.value) ? elbow_rps : -elbow_rps;
    digitalWrite(ELBOW_PULSE_PIN, HIGH);
    delay(4);
    digitalWrite(ELBOW_PULSE_PIN, LOW);
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();
    if (cmd == 'R')
    {
      Serial.write(waist.bytes, 4);
      Serial.write(shoulder.bytes, 4);
      Serial.write(elbow.bytes, 4);
      Serial.write(wrist_rotation.bytes, 4);
      Serial.write(wrist_flex.bytes, 4);
      Serial.write(hand_roll.bytes, 4);
    }
    if (cmd == 'W')
    {
      Serial.readBytes(waist_target.bytes, 4);
      Serial.readBytes(shoulder_target.bytes, 4);
      Serial.readBytes(elbow_target.bytes, 4);
      Serial.readBytes(wrist_rotation.bytes, 4);
      Serial.readBytes(wrist_flex.bytes, 4);
      Serial.readBytes(hand_roll.bytes, 4);
    }
    if (cmd == 'D')
    {
      Serial.println(waist.value, 4);
      Serial.println(shoulder.value, 4);
      Serial.println(elbow.value, 4);
      Serial.println(wrist_rotation.value, 4);
      Serial.println(wrist_flex.value, 4);
      Serial.println(hand_roll.value, 4);
    }
    if (cmd == 'T')
    {
      shoulder_target.value = 1.571;
      elbow_target.value = 1.571;
    }
    if (cmd == 'Z')
    {
      shoulder_target.value = 0;
      elbow_target.value = 0;
    }
  }
  update_arm();
}
