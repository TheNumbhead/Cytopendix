/*Code is for two stepper motors connected to a4988 stepper motor controllers operating in full step mode.
For each motor this program controls the inputs: STEP, DIR, and !ENABLE*/

/* This program alternates the stepping of the two motors, only stepping one motor at a time,
as well as alternating the direction that the motors are stepped. It was also written for flexability as opposed to efficiency*/

/*One motor controller per motor.  a4988 is a motor driver with control circuitry. */

/*ENABLE input: turns on or off all of the FET outputs. 
logic HIGH DISABLES the outputs.
logic LOW, the internal control ENABLES the outputs as required.
translator inputs STEP, DIR, and MSx, as well as the
internal sequencing logic, all remain active, independent of ENABLE*/

#define stepPin1 4 //STEP for motor 1. low to high signal corresponts to one step of the motor (signal leading edge)
#define dirPin1 5 //DIR for motor 1 determines which way the motor steps 
#define disabPin1 6 // !ENABLE pin on controller 1. 
#define stepPin2 8 //STEP for motor 2
#define dirPin2 9 //DIR for motor 2
#define disabPin2 10 //!ENABLE pin on controller 2.

//values of the direction and enable/disable outputs
boolean dir1, dir2, disab1, disab2;

// Stepping variables
const int STEPS_DEFAULT = 200;
int num_steps;
bool dir;

// Switching variables
const unsigned long EXP_DUR_DEFAULT = 8 * 3600000;
unsigned long exp_dur;
unsigned long exp_start_time;
long time_until_sample;
int samp_vol;
int wash_rate;
unsigned long samp_per;
bool run_program;

//Serial parsing
const int NUM_COMMANDS = 11;
const char commands[NUM_COMMANDS][32] = {"run", "stop", "step", "dir", "sampvol", "expdur", "sampper", "washrate", "timetosample", "timeleft", "h"};
enum COM {RUN, STOP, STEP, DIR, SAMP_VOL, EXP_DUR, SAMP_PER, WASH_RATE, TIME_TO_SAMPLE, TIME_LEFT, HELP};
char message[1024];
char command[1024];
char value_str[1024];
unsigned long value;
int message_len;
bool reading;
enum TUNIT {SECS, MS, MNS, HRS};

void setup() {
  Serial.begin(9600);//helpful when debugging
  //configuring pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(disabPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(disabPin2, OUTPUT);
  //initializing outputs
  dir1 = LOW;
  dir2 = LOW;
  //both controllers are initially disabled
  disab1 = LOW;
  disab2 = LOW;
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);
  digitalWrite(dirPin1, dir1);
  digitalWrite(dirPin2, dir2);  
  digitalWrite(disabPin1, LOW);
  digitalWrite(disabPin2, LOW);

  // Stepping
  num_steps = 0;
  dir = LOW;

  // Switching
  exp_dur = EXP_DUR_DEFAULT;
  exp_start_time = 0L;
  time_until_sample = 0L;
  samp_vol = 3;
  wash_rate = 111;
  samp_per = timeConv(5, MNS, MS);

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
    Serial.println(commands[i]);
  }
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
   delay(10);
}

void stepProgram() {
  stepping(stepPin1, dirPin1, dir, 1, 2);
  num_steps--;
  
  if (num_steps == 0) {
    Serial.println("Done stepping");
  }
}

void switchingPumpsProgram(const unsigned int delta_time) {
  // Control sampling (once every samp_per)
  if (time_until_sample <= 0) {
    stepping(stepPin1, dirPin1, LOW, sampVolToSteps(samp_vol), 2);
    Serial.print("Sampling syringe pump moved ");
    Serial.print(sampVolToSteps(samp_vol));
    Serial.println(" steps");
    time_until_sample = samp_per;
  }
  time_until_sample -= delta_time;

  // Control wash
  //                      uL / step    / (uL / ms)
  int ms_per_step = (int)(0.2543327611 / (wash_rate / 60.0 / 1000.0));
  stepping(stepPin2, dirPin2, LOW, 1, ms_per_step / 2);

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

//input parameters: 
// stepPin and dirPin select the contoller.
// nSteps is the number of motor steps.
// dir sets the direction to high or low
// stepDelay is the time between pulses in milliseconds (1ms seems to be a good minimum, for now) 
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
    exp_dur = exp_dur != 0 ? exp_dur : EXP_DUR_DEFAULT;
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
  } else if (stringEquals(command, commands[STOP])) {
    run_program = false;
    num_steps = 0;
    time_until_sample = 0;
    exp_start_time = 0;
    Serial.println("Program stopped...");
  } else if (stringEquals(command, commands[STEP])) {
    getValue();
    num_steps = value != 0 ? value : STEPS_DEFAULT;
    Serial.print("Number of steps set to ");
    Serial.println(num_steps);
    run_program = true;
    Serial.println("Stepping...");
  } else if (stringEquals(command, commands[DIR])) {
    getValue();
    dir = value;
    Serial.print("Direction set to ");
    Serial.println(dir);
  } else if (stringEquals(command, commands[SAMP_VOL])) {
    getValue();
    if (value != 0)
     samp_vol = value;
    Serial.print("Sample volume set to ");
    Serial.print(samp_vol);
    Serial.println(" uL");
  } else if (stringEquals(command, commands[EXP_DUR])) {
    getValue();
    if (value != 0)
      exp_dur = timeConv(value, HRS, MS);
    Serial.print("Experiment duration set to ");
    Serial.print(timeConv(exp_dur, MS, HRS));
    Serial.println(" hours");
  } else if (stringEquals(command, commands[SAMP_PER])) {
    getValue();
    if (value != 0)
      samp_per = timeConv(value, MNS, MS);
    Serial.print("Sample period set to ");
    Serial.print(timeConv(samp_per, MS, MNS));
    Serial.println(" minutes");
  } else if (stringEquals(command, commands[WASH_RATE])) {
    getValue();
    if (value != 0)
      wash_rate = value;
    Serial.print("Wash rate set to ");
    Serial.print(wash_rate);
    Serial.println(" uL / min");
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
