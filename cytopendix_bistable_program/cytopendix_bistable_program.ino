#define stepPin1 13 //STEP for motor 1. low to high signal corresponts to one step of the motor (signal leading edge)
#define dirPin1 12 //DIR for motor 1 determines which way the motor steps
#define stepPin2 11 //STEP for motor 2
#define dirPin2 10 //DIR for motor 2
#define stepPin3 9 //STEP for motor 3
#define dirPin3 8 //DIR for motor 3
#define stepPin4 7 //STEP for motor 4
#define dirPin4 6 //DIR for motor 4

#define MIXDELAY ((60000) / ((48) * mix_rate))

// Stepping
const int STEPS_DEFAULT = 200;
int num_steps;
int pump_to_step;
bool dir;

// Bistable characterization parameters
unsigned long exp_dur;
unsigned long exp_start_time;
unsigned long exp_pause_time;
long time_until_sample;
int samp_vol;
unsigned long samp_per;
unsigned long input_dur;
int mix_rate;
int mix_dir;
int inputStepPin;
int inputDirPin;
bool run_program;

//Serial parsing
const int NUM_COMMANDS = 19;
const char commands[NUM_COMMANDS][32] = {"run", "pause", "resume", "stop", "step1", "step2", "step3", "step4", "dir", "mixdir", "sampvol", "sampper", "inputdur", "curinput", "mixrate", "expdur", "timetosample", "timeleft", "h"};
enum COM {RUN, PAUSE, RESUME, STOP, STEP1, STEP2, STEP3, STEP4, DIR, MIX_DIR, SAMP_VOL, SAMP_PER, INPUT_DUR, CUR_INPUT, MIX_RATE, EXP_DUR, TIME_TO_SAMPLE, TIME_LEFT, HELP};
char message[1024];
char command[1024];
char value_str[1024];
unsigned long value;
int message_len;
bool reading;
enum TUNIT {SECS, MS, MNS, HRS};

void setup() {
  Serial.begin(9600);
  
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
  
  // Stepping
  num_steps = 0;
  dir = LOW;
  pump_to_step = 1;

  // Switching
  exp_dur = timeConv(8, HRS, MS);
  exp_start_time = 0L;
  exp_pause_time = 0L;
  time_until_sample = 0L;
  samp_vol = 3;
  samp_per = timeConv(5, MNS, MS);
  input_dur = timeConv(2, HRS, MS);
  mix_rate = 5;
  mix_dir = LOW;
  inputStepPin = stepPin1;
  inputDirPin = dirPin1;

  // Serial I/O
  memset(message, '\0', 1024);
  memset(command, '\0', 1024);
  memset(value_str, '\0', 1024);
  value = 0L;
  message_len = 0;
  reading = false;
  printGreeting();
}

void printGreeting() {
  Serial.println("You could type some of these commands I guess:");
  for (int i = 0; i < NUM_COMMANDS; i++) {
    Serial.print(" - ");
    if (i % 2) {
      Serial.println(commands[i]);
    } else {
      Serial.print(commands[i]);
      char tab[15 - strlen(commands[i])];
      memset(tab, ' ', 15 - strlen(commands[i]));
      Serial.print(tab);
    }
  }
  Serial.println();
}

unsigned long prev_loop_start_time;
void loop() {
  // Track time between loops
  int delta_time = millis() - prev_loop_start_time;
  prev_loop_start_time = millis();
  
  // Read Serial
  if (Serial.available()) {
    Read();
  } else {
    Parse();
  }

  // Main program
  if (run_program) {
    if (num_steps) {
      stepProgram();
    } else if (exp_start_time && millis() - exp_start_time < exp_dur) {
      switchingPumpsProgram(delta_time);
    } else {
      run_program = false;
    }
  }
  
  // For serial port delays
  if (!run_program)
   delay(5);
}

void stepProgram() {
  if (pump_to_step == 1) {
    stepping(stepPin1, dirPin1, dir, 1, 1);
  } else if (pump_to_step == 2) {
    stepping(stepPin2, dirPin2, dir, 1, 1);
  } else if (pump_to_step == 3) {
    stepping(stepPin3, dirPin3, dir, 1, 1);
  } else {
    stepping(stepPin4, dirPin4, mix_dir, 1, MIXDELAY / 2);
  }
  num_steps--;
  
  if (num_steps == 0) {
    Serial.println("Done stepping");
  }
}

void switchingPumpsProgram(const unsigned int delta_time) {
  // Switch pumps according to experiment run time
  int prev_pump_to_step = pump_to_step;
  unsigned long exp_phase = ((millis() - exp_start_time) / input_dur) % 4;
  switch (exp_phase) {
    case 0:
      pump_to_step = 1;
      inputStepPin = stepPin1;
      inputDirPin = dirPin1;
      break;
    case 1:
    case 3:
      pump_to_step = 2;
      inputStepPin = stepPin2;
      inputDirPin = dirPin2;
      break;
    case 2:
      pump_to_step = 3;
      inputStepPin = stepPin3;
      inputDirPin = dirPin3;
      break;
  }
  if (prev_pump_to_step != pump_to_step) {
    Serial.print("The current input has been set from pump ");
    Serial.print(prev_pump_to_step);
    Serial.print(" to pump ");
    Serial.println(pump_to_step);
  }
  
  // Control sampling (once every samp_per)
  if (time_until_sample <= 0) {
    stepping(inputStepPin, inputDirPin, LOW, sampVolToSteps(samp_vol), 2);
    Serial.print("Sampling syringe pump (pump ");
    Serial.print(pump_to_step);
    Serial.print(") moved ");
    Serial.print(sampVolToSteps(samp_vol));
    Serial.println(" steps");
    time_until_sample = samp_per;
  }
  time_until_sample -= delta_time;

  // Control mixer
  stepping(stepPin4, dirPin4, mix_dir, 1, MIXDELAY / 2);

  // Report upcoming sample
  reportUpcomingEvent(delta_time, time_until_sample, "next sample");

  // Report upcoming time to move output tube
  reportUpcomingEvent(delta_time, time_until_sample - samp_per / 2.0, "you should move the output tube");
}

void reportUpcomingEvent(const unsigned int delta_time, long time_until_event, const char* event_str) {
  int mins_to_event = timeConv(time_until_event, MS, MNS);
  int prev_mins_to_event = timeConv(time_until_event + delta_time, MS, MNS);
  if (mins_to_event < 2 && prev_mins_to_event >= 2) {
    Serial.print("2 minutes until ");
    Serial.println(event_str);
  } else if (mins_to_event < 1 && prev_mins_to_event >= 1) {
    Serial.print("1 minute until ");
    Serial.println(event_str);
  } else if (mins_to_event < 1) {
    int secs_to_event = timeConv(time_until_event, MS, SECS);
    int prev_secs_to_event = timeConv(time_until_event + delta_time, MS, SECS);
    if (secs_to_event < prev_secs_to_event) {
      if (secs_to_event == 30 || secs_to_event == 15 || secs_to_event <= 10) {
        Serial.print(secs_to_event);
        Serial.print(" seconds until ");
        Serial.println(event_str);      
      }
    }
  }
}

int sampVolToSteps(int samp_vol) {
  // uL / step determined experimentally
  return (int)(samp_vol / 0.2543327611);
}

void stepping(int stepPin, int dirPin, boolean dir, int nSteps, int stepDelay) {
  digitalWrite(dirPin, dir);
  for(int i = 0; i < nSteps; i++) {
    digitalWrite(stepPin, LOW);
    delay(stepDelay);
    digitalWrite(stepPin, HIGH);
    delay(stepDelay);
  }  
}

// Add each Serial read to message
void Read() {
  reading = true;
  message[message_len] = Serial.read();
  message_len++;
  memset(command, '\0', 1024);
  memset(value_str, '\0', 1024);
}

void Parse() {
  reading = false;
  if (message_len) {
    getCommandAndValueString();
    /*Serial.print("Message read: ");
    Serial.println(message);
    Serial.print("Command: ");
    Serial.println(command);
    Serial.print("Value: ");
    Serial.println(value_str);*/
    memset(message, '\0', 1024);
    message_len = 0;

    setValByCommand();
  }
}

void getCommandAndValueString() {
  bool reading_command = true;
  char delimit = ' ';
  int vi = 0;
  for (int i = 0; message[i] != '\0'; i++) {
    if (message[i] == delimit) {
      reading_command = false;
    } else if (reading_command) {
      command[i] = message[i];
    } else {
      value_str[vi] = message[i];
      vi++;
    }
  }
}

void setValByCommand() {
  if (stringEquals(command, commands[RUN])) {
    exp_start_time = millis();
    time_until_sample = 0;
    run_program = true;
    Serial.println("Program started...");
    Serial.print("Experiment duration: ");
    Serial.print(timeConv(exp_dur, MS, HRS));
    Serial.println(" hours");
    Serial.print("Sample period: ");
    Serial.print(timeConv(samp_per, MS, MNS));
    Serial.println(" minutes");
    Serial.print("Sample volume: ");
    Serial.print(samp_vol);
    Serial.println(" uL");
    Serial.print("Input duration: ");
    Serial.print(timeConv(input_dur, MS, SECS));
    Serial.println(" seconds");
    Serial.print("Mix rate: ");
    Serial.print(mix_rate);
    Serial.println(" RPM");
  } else if (stringEquals(command, commands[PAUSE])) {
    run_program = false;
    exp_pause_time = millis();
    Serial.println("Program paused...");
  } else if (stringEquals(command, commands[RESUME])) {
    exp_start_time += millis() - exp_pause_time;
    run_program = true;
    Serial.println("Experiment resumed!");
  } else if (stringEquals(command, commands[STOP])) {
    run_program = false;
    num_steps = 0;
    time_until_sample = 0;
    exp_start_time = 0;
    Serial.println("Program stopped...");
  } else if (stringEquals(command, commands[STEP1]) || stringEquals(command, commands[STEP2]) || stringEquals(command, commands[STEP3]) || stringEquals(command, commands[STEP4])) {
    getValue();
    num_steps = value != 0 ? value : STEPS_DEFAULT;
    pump_to_step = stringEquals(command, commands[STEP1]) ? 1 : stringEquals(command, commands[STEP2]) ? 2 : stringEquals(command, commands[STEP3]) ? 3 : 4;
    Serial.print("Number of steps set to ");
    Serial.println(num_steps);
    run_program = true;
    Serial.print("Stepping Pump ");
    Serial.print(pump_to_step);
    Serial.println("...");
  } else if (stringEquals(command, commands[DIR])) {
    getValue();
    dir = value;
    Serial.print("Direction set to ");
    Serial.println(dir);
  } else if (stringEquals(command, commands[MIX_DIR])) {
    getValue();
    mix_dir = value;
    Serial.print("Mixer direction set to ");
    Serial.println(dir);
  } else if (stringEquals(command, commands[SAMP_VOL])) {
    getValue();
    if (value != 0)
     samp_vol = value;
    Serial.print("Sample volume set to ");
    Serial.print(samp_vol);
    Serial.println(" uL");
  } else if (stringEquals(command, commands[SAMP_PER])) {
    getValue();
    if (value != 0)
      samp_per = timeConv(value, MNS, MS);
    Serial.print("Sample period set to ");
    Serial.print(timeConv(samp_per, MS, MNS));
    Serial.println(" minutes");
  } else if (stringEquals(command, commands[INPUT_DUR])) {
    getValue();
    if (value != 0)
      input_dur = timeConv(value, SECS, MS);
    Serial.print("Input duration set to ");
    Serial.print(timeConv(input_dur, MS, SECS));
    Serial.println(" secs");
  } else if (stringEquals(command, commands[CUR_INPUT])) {
    Serial.print("The current input is set to pump ");
    Serial.println(pump_to_step);
  } else if (stringEquals(command, commands[MIX_RATE])) {
    getValue();
    if (value != 0)
      mix_rate = value;
    Serial.print("Mix rate set to ");
    Serial.print(mix_rate);
    Serial.println(" RPM");
  } else if (stringEquals(command, commands[EXP_DUR])) {
    getValue();
    if (value != 0)
      exp_dur = timeConv(value, HRS, MS);
    Serial.print("Experiment duration set to ");
    Serial.print(timeConv(exp_dur, MS, HRS));
    Serial.println(" hours");
  } else if (stringEquals(command, commands[TIME_TO_SAMPLE])) {
    printTimeLeft(time_until_sample);
  } else if (stringEquals(command, commands[TIME_LEFT])) {
    printTimeLeft(exp_dur - (millis() - exp_start_time));
  } else if (stringEquals(command, commands[HELP])) {
    printGreeting();
  }
}

bool stringEquals(const char s1[], const char s2[]) {
  int s1_len, s2_len = 0;
  for (;s1[s1_len] != '\0'; s1_len++);
  for (;s2[s2_len] != '\0'; s2_len++);
  if (s1_len != s2_len ) {
    return false;
  }
  
  for (int i = 0; s1[i] != '\0' && s2[i] != '\0'; i++) {
    if (s1[i] != s2[i]) {
      return false;
    }
  }
  return true;
}

int getValue() {
  value = 0;
  for (int i = 0; value_str[i] != '\0'; i++) {
    value = value*10 + value_str[i] - '0';
  }
}

unsigned long timeConv(unsigned long val, enum TUNIT t1, enum TUNIT t2) {
  // Convert val to ms
  switch (t1) {
    case HRS: val *= 3600000; break;
    case MNS: val *= 60000; break;
    case SECS: val *= 1000; break;
  }
  // Convert val to output time unit
  switch (t2) {
    case HRS: val /= 3600000; break;
    case MNS: val /= 60000; break;
    case SECS: val /= 1000; break;
  }
  return val;
}

void printTimeLeft(unsigned long ms) {
  Serial.print("Time left: ");
  Serial.print(timeConv(ms, MS, HRS));
  Serial.print(" hours, ");
  ms -= timeConv(timeConv(ms, MS, HRS), HRS, MS);
  Serial.print(timeConv(ms, MS, MNS));
  Serial.print(" minutes, and ");
  ms -= timeConv(timeConv(ms, MS, MNS), MNS, MS);
  Serial.print(timeConv(ms, MS, SECS));
  Serial.println(" seconds");
}
