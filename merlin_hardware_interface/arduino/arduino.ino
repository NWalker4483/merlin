#define WAIST_PULSE_PIN 7
#define WAIST_DIR_PIN 6
#define WAIST_BRAKE_PIN 8
#define WAIST_SPR 200
#define WAIST_REDUCTION 48 // 1:x

#define SHOULDER_PULSE_PIN 5
#define SHOULDER_DIR_PIN 4
#define SHOULDER_BRAKE_PIN 9
#define SHOULDER_SPR 200
#define SHOULDER_REDUCTION 48 // 1:x

#define ELBOW_PULSE_PIN 3
#define ELBOW_DIR_PIN 2
#define ELBOW_BRAKE_PIN 10
#define ELBOW_SPR 200
#define ELBOW_REDUCTION 48 // 1:x

#define AXIS_4_PULSE_PIN 11
#define AXIS_4_DIR_PIN 12

#define AXIS_5_PULSE_PIN 13
#define AXIS_5_DIR_PIN 14

#define AXIS_6_PULSE_PIN 15
#define AXIS_6_DIR_PIN 16

#define ONE_DEGREE 0.0175
#define PI 3.14

#define WAIST_RAD_PER_STEP (((2 * PI) / WAIST_SPR) / WAIST_REDUCTION)
#define SHOULDER_RAD_PER_STEP (((2 * PI) / SHOULDER_SPR) / SHOULDER_REDUCTION)
#define ELBOW_RAD_PER_STEP (((2 * PI) / ELBOW_SPR) / ELBOW_REDUCTION)
#define WRIST_ROT_RAD_PER_STEP 0
#define WRIST_FLEX_RAD_PER_STEP 0
#define HAND_ROLL_RAD_PER_STEP 0

union open_float
{
  char bytes[4];
  float value = 0;
};

open_float waist, shoulder, elbow, wrist_roll, wrist_flex, hand_roll;
open_float waist_target, shoulder_target, elbow_target, wrist_roll_target, wrist_flex_target, hand_roll_target;

int pulse_pins[6] = {WAIST_PULSE_PIN, SHOULDER_PULSE_PIN, ELBOW_PULSE_PIN, AXIS_4_PULSE_PIN, AXIS_5_PULSE_PIN, AXIS_6_PULSE_PIN};
int dir_pins[6] = {WAIST_DIR_PIN, SHOULDER_DIR_PIN, ELBOW_DIR_PIN, AXIS_4_DIR_PIN, AXIS_5_DIR_PIN, AXIS_6_DIR_PIN};
bool wrist_first = false;

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

  pinMode(AXIS_4_PULSE_PIN, OUTPUT);
  pinMode(AXIS_4_DIR_PIN, OUTPUT);

  pinMode(AXIS_5_PULSE_PIN, OUTPUT);
  pinMode(AXIS_5_DIR_PIN, OUTPUT);

  pinMode(AXIS_6_PULSE_PIN, OUTPUT);
  pinMode(AXIS_6_DIR_PIN, OUTPUT);
}

void load_saved_state()
{
  waist.value = 0;
  shoulder.value = 0;
  elbow.value = 0;
  wrist_roll.value = 0;
  wrist_flex.value = 0;
  hand_roll.value = 0;
}

void set_brakes(bool lock)
{
  digitalWrite(WAIST_BRAKE_PIN, lock ? HIGH : LOW);
  digitalWrite(SHOULDER_BRAKE_PIN, lock ? HIGH : LOW);
  digitalWrite(ELBOW_BRAKE_PIN, lock ? HIGH : LOW);
  delay(500);
}

void setup()
{
  Serial.begin(115200);

  setup_pins();

  load_saved_state();

  set_brakes(false);

  Serial.println("MERLIN MR6500 Controller Started");
}

void update_arm_state()
{
  waist.value = ;
  shoulder.value = ;
  elbow.value = ;
  wrist_roll.value = ;
  wrist_flex.value = ;
  hand_roll.value = ;
}

void update_arm_controls()
{
  int axis_steps[6] = {0, 0, 0, 0, 0, 0};

  if (abs(waist.value - waist_target.value) > (ONE_DEGREE / 2))
  {
    axis_steps[0] = (waist.value > waist_target.value) ? -1 : 1; // Flipped Axis
  }

  if (abs(shoulder.value - shoulder_target.value) > (ONE_DEGREE / 2))
  {
    axis_steps[1] = (shoulder.value > shoulder_target.value) ? 1 : -1;
  }

  if (abs(elbow.value - elbow_target.value) > (ONE_DEGREE / 2))
  {
    axis_steps[2] = (elbow.value > elbow_target.value) ? 1 : -1;
  }
  // Start Update Wrist
  if (wrist_first)
  {
    if (abs(wrist_roll.value - wrist_roll_target.value) > (ONE_DEGREE / 2))
    {
      if (wrist_roll.value > wrist_roll_target.value)
      {
        axis_steps[3], axis_steps[4], axis_steps[5] = 1;
      }
      else
      {
        axis_steps[3], axis_steps[4], axis_steps[5] = -1;
      }
    }
    if (abs(wrist_flex.value - wrist_flex_target.value) > (ONE_DEGREE / 2))
    {
      if ((wrist_flex.value > wrist_flex_target.value) and axis_steps[0] != 1)
      {
        axis_steps[4] += 1;
        axis_steps[5] += 1;
      }
      else if ((wrist_flex.value < wrist_flex_target.value) and axis_steps[0] != -1)
      {
        axis_steps[4] -= 1;
        axis_steps[5] -= 1;
      }
    }
    if (abs(hand_roll.value - hand_roll_target.value) > (ONE_DEGREE / 2))
    {
      if ((hand_roll.value > hand_roll_target.value) and axis_steps[1] != 1)
      {
        axis_steps[5] += 1;
      }
      if ((hand_roll.value < hand_roll_target.value) and axis_steps[1] != -1)
      {
        axis_steps[5] -= 1;
      }
    }
  }
  else
  {

    if (abs(hand_roll.value - hand_roll_target.value) > (ONE_DEGREE / 2))
    {
      if ((hand_roll.value > hand_roll_target.value) and axis_steps[1] != 1)
      {
        axis_steps[5] += 1;
      }
      if ((hand_roll.value < hand_roll_target.value) and axis_steps[1] != -1)
      {
        axis_steps[5] -= 1;
      }
    }
    if (abs(wrist_flex.value - wrist_flex_target.value) > (ONE_DEGREE / 2))
    {
      if ((wrist_flex.value > wrist_flex_target.value) and axis_steps[0] != 1)
      {
        axis_steps[4] += 1;
        axis_steps[5] += 1;
      }
      else if ((wrist_flex.value < wrist_flex_target.value) and axis_steps[0] != -1)
      {
        axis_steps[4] -= 1;
        axis_steps[5] -= 1;
      }
    }

    if (abs(wrist_roll.value - wrist_roll_target.value) > (ONE_DEGREE / 2))
    {
      if (wrist_roll.value > wrist_roll_target.value)
      {
        axis_steps[3], axis_steps[4], axis_steps[5] = 1;
      }
      else
      {
        axis_steps[3], axis_steps[4], axis_steps[5] = -1;
      }
    }
  }
  wrist_first = !wrist_first; // Swap ordering each time step
  // Finished Update Wrist //

  // Minimize delay by stepping all motors at once
  for (int i = 0; i < 6; i++)
  {
    if (axis_steps[i] != 0)
    {
      digitalWrite(dir_pins[i], (axis_steps[i] > 0) ? HIGH : LOW);
      digitalWrite(pulse_pins[i], HIGH);
    }
  }
  delay(4);
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(pulse_pins[i], LOW);
  }
}

void handle_commands()
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();
    if (cmd == 'R')
    {
      Serial.write(waist.bytes, 4);
      Serial.write(shoulder.bytes, 4);
      Serial.write(elbow.bytes, 4);
      Serial.write(wrist_roll.bytes, 4);
      Serial.write(wrist_flex.bytes, 4);
      Serial.write(hand_roll.bytes, 4);
    }
    if (cmd == 'W')
    {
      Serial.readBytes(waist_target.bytes, 4);
      Serial.readBytes(shoulder_target.bytes, 4);
      Serial.readBytes(elbow_target.bytes, 4);
      Serial.readBytes(wrist_roll.bytes, 4);
      Serial.readBytes(wrist_flex.bytes, 4);
      Serial.readBytes(hand_roll.bytes, 4);
    }
    if (cmd == 'D')
    {
      Serial.println(waist.value, 4);
      Serial.println(shoulder.value, 4);
      Serial.println(elbow.value, 4);
      Serial.println(wrist_roll.value, 4);
      Serial.println(wrist_flex.value, 4);
      Serial.println(hand_roll.value, 4);
    }
    if (cmd == 'T')
    {
      wrist_roll_target.value = 1.5708;
    }
    if (cmd == 'Z')
    {
      wrist_roll_target.value = 0;
    }
  }
}

void loop()
{
  handle_commands();
  update_arm_controls();
  update_arm_state();
}
