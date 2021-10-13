#define CALIBRATING false

#define WAIST_PULSE_PIN 7
#define WAIST_DIR_PIN 6
#define WAIST_BRAKE_PIN 8
#define WAIST_END_PIN 9

#define SHOULDER_PULSE_PIN 5
#define SHOULDER_DIR_PIN 4
#define SHOULDER_BRAKE_PIN 7
#define SHOULDER_END_PIN 8

#define ELBOW_PULSE_PIN 3
#define ELBOW_DIR_PIN 2
#define ELBOW_BRAKE_PIN 10
#define ELBOW_END_PIN 11

#define HEAD_ROLL_PULSE_PIN 14
#define HEAD_ROLL_DIR_PIN 6

#define HEAD_PITCH_PULSE_PIN 15
#define HEAD_PITCH_DIR_PIN 6

#define HEAD_YAW_PULSE_PIN 16
#define HEAD_YAW_DIR_PIN 6

int waist_steps = 0;
int waist_target = 0;
unsigned long waist_last_pulse = 0;
int waist_delay = 3;

int shoulder_steps = 0;
int shoulder_target = 0;
unsigned long shoulder_last_pulse = 0;
int shoulder_delay = 3;

int elbow_steps = 0;
int elbow_target = 0;
unsigned long elbow_last_pulse = 0;
int elbow_delay = 3;
// union {
//    byte array[4];
//    float bigNum;
// } myUnion;


// // bunch of code...

//   myUnion.bigNum = 3.14;

//   for (int i = 0; i < sizeof(float); i++) {
//       SendSomeplace(myUnion.array[i]);   // Send to wherever
//   }

// // on the receiving end...

//   for (int i = 0; i < sizeof(float); i++) {
//       myUnion.array[i] = readBytes();      // Whatever is takes to read the incoming bytes
//   }
//   float pi = myUnion.bigNum;
  
void setup_pins(){
  pinMode(WAIST_PULSE_PIN, OUTPUT);
  pinMode(WAIST_DIR_PIN, OUTPUT);
  pinMode(WAIST_BRAKE_PIN, OUTPUT);
  
  pinMode(SHOULDER_PULSE_PIN, OUTPUT);
  pinMode(SHOULDER_DIR_PIN, OUTPUT);
  pinMode(SHOULDER_BRAKE_PIN, OUTPUT);
  
  pinMode(ELBOW_PULSE_PIN, OUTPUT);
  pinMode(ELBOW_DIR_PIN, OUTPUT);
  pinMode(ELBOW_BRAKE_PIN, OUTPUT);

  pinMode(HEAD_ROLL_PULSE_PIN, OUTPUT);
  pinMode(HEAD_ROLL_DIR_PIN, OUTPUT);

  pinMode(HEAD_PITCH_PULSE_PIN, OUTPUT);
  pinMode(HEAD_PITCH_DIR_PIN, OUTPUT);

  pinMode(HEAD_YAW_PULSE_PIN, OUTPUT);
  pinMode(HEAD_YAW_DIR_PIN, OUTPUT);
}

void setup_arm(){
  setup_pins();
}
  
void update_arm(){
  unsigned long curr_time = millis();
     Serial.print(waist_steps);
     Serial.print(" ");
     Serial.println(waist_target);
     
  if (abs(waist_steps - waist_target) != 0){
    if (waist_steps < waist_target){
      digitalWrite(WAIST_DIR_PIN, HIGH);
      waist_steps += 1;
      } else {
      digitalWrite(WAIST_DIR_PIN, LOW);
      waist_steps -= 1; 
     }
    digitalWrite(WAIST_PULSE_PIN, HIGH);
    delay(waist_delay);
    digitalWrite(WAIST_PULSE_PIN, LOW);
    }

     if (abs(shoulder_steps - shoulder_target) != 0){
    if (shoulder_steps < shoulder_target){
      digitalWrite(SHOULDER_DIR_PIN, HIGH);
      shoulder_steps += 1;
      } else {
      digitalWrite(SHOULDER_DIR_PIN, LOW);
      shoulder_steps -= 1; 
     }
    digitalWrite(SHOULDER_PULSE_PIN, HIGH);
    delay(shoulder_delay);
    digitalWrite(SHOULDER_PULSE_PIN, LOW);
    }

    if (abs(elbow_steps - elbow_target) != 0){
    if (elbow_steps < elbow_target){
      digitalWrite(ELBOW_DIR_PIN, HIGH);
      elbow_steps += 1;
      } else {
      digitalWrite(ELBOW_DIR_PIN, LOW);
      elbow_steps -= 1; 
     }
    digitalWrite(ELBOW_PULSE_PIN, HIGH);
    delay(elbow_delay);
    digitalWrite(ELBOW_PULSE_PIN, LOW);
    }
  }

void setup() {
  Serial.begin(9600);
  setup_arm();
}

const int dance_length = 7;
int dance[dance_length][3]  = {{0,0,0}, {1500,3000,100}, {3000,3000,500},{3000,6000,-2000},{3000,6000,-500},{1000,6000,0}, {0,0,0} };
int dance_step = 0; 
bool done = false;

void loop() {
  unsigned long curr_time = millis();
  update_arm();
  done = true;
  
  if (waist_target - waist_steps != 0){
    done = false;
  }
  
  if (shoulder_target - shoulder_steps != 0){
    done = false;
  }
  
  if (elbow_target - elbow_steps != 0){
    done = false;
  }
 
   if (done){
    delay(100);
      dance_step += 1;
      waist_target = dance[dance_step-1][0];
      shoulder_target = dance[dance_step-1][1];
      elbow_target = dance[dance_step -1][2];
    }
   
   if (dance_step > dance_length){
    dance_step = 0; 
    while(1);
    }
}

void home_axis(int dir_pin, int pulse_pin, int end_pin){}

// void home(){
//  digitalWrite(WAIST_DIR_PIN, LOW);
//  digitalWrite(SHOULDER_DIR_PIN, LOW);
//  digitalWrite(ELBOW_DIR_PIN, LOW);
//  while(1){
//    bool homed = true;
//    if (!digitalRead(WAIST_END_PIN) == HIGH){
//      digitalWrite(WAIST_PULSE_PIN, HIGH);
//      homed = false;
//      }
//      if (!digitalRead(SHOULDER_END_PIN) == HIGH){
//      digitalWrite(SHOULDER_PULSE_PIN, HIGH);
//      homed = false;
//      }
//      if (!digitalRead(ELBOW_END_PIN) == HIGH){
//      digitalWrite(ELBOW_PULSE_PIN, HIGH);
//      homed = false;
//      }
//    delay(3);
//    digitalWrite(WAIST_PULSE_PIN, LOW);
//    digitalWrite(SHOULDER_PULSE_PIN, LOW);
//    digitalWrite(ELBOW_PULSE_PIN, LOW);
//    if(homed){
//      break;
//     }
//    }
//  }