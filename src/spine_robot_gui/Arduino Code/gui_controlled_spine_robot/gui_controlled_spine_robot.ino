#define STBY1 13
#define AIN1 12
#define AIN2 11
#define BIN1 10
#define BIN2 9
#define STBY2 8
#define CIN1 7
#define CIN2 6
#define DIN1 5
#define DIN2 4

#define encoder1A 3
#define encoder1B 2
#define encoder2A 18
#define encoder2B 19
#define encoder3A 20
#define encoder3B 22
#define encoder4A 21
#define encoder4B 23

volatile int encoder1Pos = 0;
volatile int encoder2Pos = 0;
volatile int encoder3Pos = 0;
volatile int encoder4Pos = 0;
int encoderCountsOneRev = 596;

char GUIInput[10]; // Character array to hold individual bytes sent from GUI via serial
int parsedInputValues[3]; // Three-integer array to hold values to send to robot specifying requested robot action, parsed from GUIInput
int pos = 0; // Index counter for GUIInput parsing

bool stopRequested = false; // True stop is requested from GUI, checked using checkStopRequested function
bool functionRunning = false; // True if one of the larger composite functions (turn12, forwardDrive, or backwardDrive) is running


void doEncoder1() {
  if ((digitalRead(encoder1B)) == digitalRead(encoder1A)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void doEncoder2() {
  if ((digitalRead(encoder2B)) == digitalRead(encoder2A)) {
    encoder2Pos++;
  } else {
    encoder2Pos--;
  }
}

void doEncoder3() {
  if ((digitalRead(encoder3B)) == digitalRead(encoder3A)) {
    encoder3Pos++;
  } else {
    encoder3Pos--;
  }
}

void doEncoder4() {
  if ((digitalRead(encoder4B)) == digitalRead(encoder4A)) {
    encoder4Pos++;
  } else {
    encoder4Pos--;
  }
}



void setup(){
  Serial.begin(115200);
  
  pinMode(STBY1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY2, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
  pinMode(encoder3A, INPUT);
  pinMode(encoder3B, INPUT);
  pinMode(encoder4A, INPUT);
  pinMode(encoder4B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1A), doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), doEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3A), doEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4A), doEncoder4, CHANGE);
  digitalWrite(STBY1, HIGH);
  digitalWrite(STBY2, HIGH);

  Serial.println("false");
}



// Move specific motor at given speed and direction
// motor: 1 = motor A, 2 = motor B
// speed: 0 = off, 255 = full speed
// cw: 1 = clockwise, 2 = counter-clockwise
void move(int motor, int speed, int cw) {
  digitalWrite(STBY1, HIGH); // Disable standby

  if (motor == 1) { // Motor A
    if (cw == 1) { // CW
      digitalWrite(AIN1, LOW);
      analogWrite(AIN2, 255-speed);
    } 
    else { // CCW
      analogWrite(AIN1, speed);
      digitalWrite(AIN2, HIGH);
    }
  } 
  
  else if (motor == 2) { // Motor B
    if(cw == 1) { // CW
      digitalWrite(BIN1, LOW);
      analogWrite(BIN2, 255-speed);
    } 
    else { //CCW
      analogWrite(BIN1, speed);
      digitalWrite(BIN2, HIGH);
    }
  } 
  
  else if(motor == 3){ // Motor C
    if(cw == 1) { // CW
      digitalWrite(CIN1, LOW);
      analogWrite(CIN2, 255-speed);
    } 
    else { // CCW
      analogWrite(CIN1, speed);
      digitalWrite(CIN2, HIGH);
    }
  } 
  
  else if(motor == 4){ // Motor D
    if(cw == 1) { // CW
      digitalWrite(DIN1, LOW);
      analogWrite(DIN2, 255-speed);
    } 
    else { // CCW
      analogWrite(DIN1, speed);
      digitalWrite(DIN2, HIGH);
    }
  }

}


// Releases clamp 1 (top clamp)
void turn1(int speed, int degree, int cw) {
  // Executes only if running as independent function, not within larger composite function
  if (!functionRunning) {
    Serial.println("true"); // Indicates task is running
  }

  Serial.print("Turning Motor 1 : ");
  Serial.print(degree);
  Serial.println(" degrees");
  int turning = encoderCountsOneRev * (degree / 360.0);
  Serial.print("Turning: ");
  Serial.println(turning);
  int  initialEncoderPos = encoder1Pos;
  move(1, speed, cw);
  while (abs(encoder1Pos - initialEncoderPos) < turning) {
    move(1, speed, cw);
    Serial.print("value diff: ");
    Serial.println(abs(encoder1Pos - initialEncoderPos));
    checkStopRequested();
    if (stopRequested) {
      break;
    }
  }
  stop(1);

  if (!functionRunning) {
    delay(200);
    stopRequested = false; // Resets stopRequested
    Serial.println("false"); // Resets GUI back to initial state
  }
}

// Releases clamp 2 (bottom clamp)
void turn2(int speed, int degree, int cw) {
  // Executes only if running as independent function, not within larger composite function
  if (!functionRunning) {
    Serial.println("true"); // Indicates task is running
  }

  Serial.print("Turning Motor 2 : ");
  Serial.print(degree);
  Serial.println(" degrees");
  int turning = encoderCountsOneRev * (degree / 360.0);
  Serial.print("Turning: ");
  Serial.println(turning);
  int  initialEncoderPos = encoder2Pos;
  move(2, speed, cw);
  while (abs(encoder2Pos - initialEncoderPos) < turning) {
    move(2, speed, cw);
    Serial.print("value diff: ");
    Serial.println(abs(encoder2Pos - initialEncoderPos));
    checkStopRequested();
    if (stopRequested) {
      break;
    }
  }
  stop(2);
  
  if (!functionRunning) {
    delay(200);
    stopRequested = false; // Resets stopRequested
    Serial.println("false"); // Resets GUI back to initial state
  }
}


// Contracts or extends needle/screw (contract: cw = 1 = clockwise; extend: cw = 2 = counter-clockwise)
void turn34(int speed, int degree, int cw) {
  // Executes only if running as independent function, not within larger composite function
  if (!functionRunning) {
    Serial.println("true"); // Indicates task is running
  }

  stop(3);
  stop(4);
  Serial.print("Turning Motor 3 and 4 : ");
  Serial.print(degree);
  Serial.println(" degrees");
  int turning = encoderCountsOneRev * (degree / 360.0);
  int initial4EncoderPos = encoder4Pos;
  int initial3EncoderPos = encoder3Pos;
  move(4, speed, cw);
  move(3, speed, cw);
  bool motor4 = true;
  bool motor3 = true;
  while(motor4 || motor3) {
    if (motor4) {
      if (abs(encoder4Pos - initial4EncoderPos) >= turning) {
        stop(4);
        stop(3);
        motor4 = false;
        motor3 = false;
      } else {
        move(4, speed, cw); // Keep running motor 4
        checkStopRequested();
        if (stopRequested) {
          break;
        }
      }
    }

    if (motor3) {
      if (abs(encoder3Pos - initial3EncoderPos) >= turning) {
        stop(3);
        stop(4);
        motor3 = false;
        motor4 = false;
      } else {
        move(3, speed, cw); // Keep running motor 3
        checkStopRequested();
        if (stopRequested) {
          break;
        }
      }
    }
  }
  stop(3);
  stop(4);
  
  if (!functionRunning) {
    delay(200);
    stopRequested = false; // Resets stopRequested
    Serial.println("false"); // Resets GUI back to initial state
  }
}


// Tightens clamp 1 (top clamp)
void clamp1(int speed) {
  // Executes only if running as independent function, not within larger composite function
  if (!functionRunning) {
    Serial.println("true"); // Indicates task is running
  }

  stop(1);
  Serial.print("Clamp top motor : ");
  bool motor1 = true;
  static int lastEncoder1Pos = encoder1Pos;
  static unsigned long lastCheckTime = millis();
  unsigned long currentTime = millis();
  move(1, speed, 2);
  while (motor1) {
    currentTime = millis();
    if (motor1) {
      // Check every 200 ms if encoder2Pos hasn't changed
      if (currentTime - lastCheckTime > 50) {
        if (abs(encoder1Pos - lastEncoder1Pos) < 2) {  // Stalled (no significant movement)
          Serial.println("Clamp 1 done");
          stop(1);
          motor1 = false;
        } else {
          lastEncoder1Pos = encoder1Pos;
          lastCheckTime = currentTime;
          checkStopRequested();
          if (stopRequested) {
            break;
          }
        }
      }
    }
  }
  stop(1);
  
  if (!functionRunning) {
    delay(200);
    stopRequested = false; // Resets stopRequested
    Serial.println("false"); // Resets GUI back to initial state
  }
}

// Tightens clamp 2 (bottom clamp)
void clamp2(int speed) {
  // Executes only if running as independent function, not within larger composite function
  if (!functionRunning) {
    Serial.println("true"); // Indicates task is running
  }

  stop(2);
  Serial.print("Clamp bottom motor : ");
  bool motor2 = true;
  static int lastEncoder2Pos = encoder2Pos;
  static unsigned long lastCheckTime = millis();
  unsigned long currentTime = millis();
  move(2, speed, 2);
  while (motor2) {
    currentTime = millis();
    if (motor2) {
      // Check every 200 ms if encoder2Pos hasn't changed
      if (currentTime - lastCheckTime > 50) {
        if (abs(encoder2Pos - lastEncoder2Pos) < 2) {  // Stalled (no significant movement)
          Serial.println("Clamp 2 done");
          stop(2);
          motor2 = false;
        } else {
          lastEncoder2Pos = encoder2Pos;
          lastCheckTime = currentTime;
          checkStopRequested();
          if (stopRequested) {
            break;
          }
        }
      }
    }
  }
  stop(2);
  
  if (!functionRunning) {
    delay(200);
    stopRequested = false; // Resets stopRequested
    Serial.println("false"); // Resets GUI back to initial state
  }
}


// Runs "Release All Clamps" task in GUI
void turn12(int speed, int degree, int cw) {
  functionRunning = true; // Indicates task is running
  Serial.println("true");
  checkStopRequested();
  if (!stopRequested) {
    turn1(speed, degree, cw);
  }
  checkStopRequested();
  if (!stopRequested) {
    turn2(speed, degree, cw);
  }
  delay(200);
  stopRequested = false; // Resets stopRequested
  Serial.println("false"); // Resets GUI back to initial state
  functionRunning = false;
}

// Runs "Forward Nedle Drive" task in GUI for given number of cycles and given speed
void forwardDrive(int cycles, int speed) {
  functionRunning = true; // Indicates task is running
  Serial.println("true");
  for (int i = 0; i < cycles; i++) {
    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn34(speed, 360, 2); // Extend
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    clamp1(speed); // Tighten top clamp
    stop(1);
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn2(speed, 270, 1); // Release bottom clamp
    stop(2);
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn34(speed, 360, 1); // Contract
    
    delay(200);
    stop(3);
    stop(4);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    clamp2(speed); // Tighten bottom clamp
    stop(1);
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn1(speed, 270, 1); // Release top clamp
    delay(200);
    stop(1);

    Serial.println(i+1);
  }
  delay(200);
  stopRequested = false; // Resets stopRequested
  Serial.println("false"); // Resets GUI back to initial state
  functionRunning = false;
}

// Runs "Backward Nedle Drive" task in GUI for given number of cycles and given speed
void backwardDrive(int cycles, int speed) {
  functionRunning = true; // Indicates task is running
  Serial.println("true");
  for (int i = 0; i < cycles; i++) {
    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn34(speed, 360, 1); // Contract
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    clamp1(speed); // Tighten top clamp
    stop(1);
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn2(speed, 270, 1); // Release bottom clamp
    stop(2);
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn34(speed, 360, 2); // Extend
    
    delay(200);
    stop(3);
    stop(4);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    clamp2(speed); // Tighten bottom clamp
    stop(1);
    delay(200);

    checkStopRequested();
    if (stopRequested) {
      break;
    }
    turn1(speed, 270, 1); // Release top clamp
    delay(200);
    stop(1);

    Serial.println(i+1);
  }
  delay(200);
  stopRequested = false; // Resets stopRequested
  Serial.println("false"); // Resets GUI back to initial state
  functionRunning = false;
}


// Stops motor(s)
void stop(int motor) {
  if (motor == 1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else if (motor == 2) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else if (motor == 3) {
    digitalWrite(CIN1, LOW);
    digitalWrite(CIN2, HIGH);
  } else if (motor == 4) {
    digitalWrite(DIN1, LOW);
    digitalWrite(DIN2, HIGH);
  }
}

// Checks if stop requested from GUI
void checkStopRequested() {
  while (Serial.available()) {
    char stopByte = Serial.read();
    if (stopByte == 'S') {
      stopRequested = true;
    }
  }
}



void loop() {
  stop(1);
  stop(2);
  stop(3);
  stop(4);

  // Runs while data is available in the serial buffer
  while (Serial.available()) {
    char commandByte = Serial.read();

    // Parses input values by comma once last byte of serial data has been read, places values into parsedInputValues array
    if (commandByte == '\n') {
      GUIInput[pos] = '\0';

      char* token = strtok(GUIInput, ",");
      int i = 0;
      while (token != NULL && i < 3) {
        parsedInputValues[i++] = atoi(token);
        token = strtok(NULL, ",");
      }

      for (int j = 0; j < i; j++) {
        Serial.print("Value ");
        Serial.print(j);
        Serial.print(": ");
        Serial.println(parsedInputValues[j]);
      }

      // Executes task based on first value of parsedInputValues array
      switch (parsedInputValues[0]) { // remove delays
        case 1:
          // Tightens top clamp (clamp 1)
          clamp1(150);
          break;
        case 2:
          // Releases top clamp (clamp 1)
          turn1(200, 270, 1);
          break;
        case 3:
          // Tightens bottom clamp (clamp 2)
          clamp2(150);
          break;
        case 4:
          // Releases bottom clamp (clamp 2)
          turn2(200, 270, 1);
          break;
        case 5:
          // Contract screw
          turn34(250, 720, 1);
          break;
        case 6:
          // Extend screw
          turn34(250, 711, 2);
          break;
        case 7:
          // Release all clamps
          turn12(200, 270, 1);
          break;
        
        // Forward and backward drive take as arguments number of cycles and speed, represented by the second and third values of the parsedInputValues array
        case 8:
          forwardDrive(parsedInputValues[1], parsedInputValues[2]);
          break;
        case 9:
          backwardDrive(parsedInputValues[1], parsedInputValues[2]);
          break;
      }

      pos = 0;
    }

    // Adds bytes/characters from the serial buffer to GUIInput array
    else {
      if (pos < sizeof(GUIInput) - 1) {
        GUIInput[pos++] = commandByte;
      }
      else {
        pos = 0;
      }
    }
  }
}