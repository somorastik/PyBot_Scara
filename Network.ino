// PyBotArm Robotic Arm scara robot project 
// JJROBOTS
// License: GPL v2
// Network functions (ESP module)
// And message functions (messages from the APP)
#include "pins.h"

int ESPwait(String stopstr, int timeout_secs)
{
  String response;
  bool found = false;
  char c;
  long timer_init;
  long timer;

  timer_init = millis();
  while (!found) {
    timer = millis();
    if (((timer - timer_init) / 1000) > timeout_secs) { // Timeout?
      SerialUSB.println("!Timeout!");
      return 0;  // timeout
    }
    if (Serial1.available()) {
      c = Serial1.read();
      SerialUSB.print(c);
      response += c;
      if (response.endsWith(stopstr)) {
        found = true;
        delay(10);
        Serial1.flush();
        SerialUSB.println();
      }
    } // end Serial1_available()
  } // end while (!found)
  return 1;
}

// getMacAddress from ESP wifi module
int ESPgetMac()
{
  char c1, c2;
  bool timeout = false;
  long timer_init;
  long timer;
  uint8_t state = 0;
  uint8_t index = 0;

  timer_init = millis();
  while (!timeout) {
    timer = millis();
    if (((timer - timer_init) / 1000) > 5) // Timeout?
      timeout = true;
    if (Serial1.available()) {
      c2 = c1;
      c1 = Serial1.read();
      SerialUSB.print(c1);
      switch (state) {
        case 0:
          if (c1 == ':')
            state = 1;
          break;
        case 1:
          if (c1 == '\r') {
            MAC.toUpperCase();
            state = 2;
          }
          else {
            if ((c1 != '"') && (c1 != ':'))
              MAC += c1;  // Uppercase
          }
          break;
        case 2:
          if ((c2 == 'O') && (c1 == 'K')) {
            SerialUSB.println();
            Serial1.flush();
            return 1;  // Ok
          }
          break;
      } // end switch
    } // Serial_available
  } // while (!timeout)
  SerialUSB.println("!Timeout!");
  Serial1.flush();
  return -1;  // timeout
}

int ESPsendCommand(String command, String stopstr, int timeout_secs)
{
  Serial1.println(command);
  ESPwait(stopstr, timeout_secs);
  delay(250);
}

int32_t ExtractParamInt4b(uint8_t pos) {
  union {
    unsigned char Buff[4];
    int32_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 3];
  u.Buff[1] = (unsigned char)MsgBuffer[pos + 2];
  u.Buff[2] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[3] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}

int16_t ExtractParamInt2b(uint8_t pos) {
  union {
    unsigned char Buff[2];
    int16_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[1] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}

// Messgase: 8 channels (16 bits)
void MsgRead()
{
  uint8_t i;
  // New bytes available to process?
  long t0 = micros();
  // Max 5ms reading messages...
  while ((Serial1.available() > 0)&&((micros()-t0)<5000)) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i = 0; i < (MSGMAXLEN - 1); i++) {
      MsgBuffer[i] = MsgBuffer[i + 1];
    }
    MsgBuffer[MSGMAXLEN - 1] = (unsigned char)Serial1.read();
    //SerialUSB.print((char)MsgBuffer[MSGMAXLEN-1]);
    ParseMsg(1);
  }
}

// Read messages from USB
// Message: 8 channels (16 bits)
void USBMsgRead()
{
  uint8_t i;
  // New bytes available to process?
  while (SerialUSB.available() > 0) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i = 0; i < (MSGMAXLEN - 1); i++) {
      MsgBuffer[i] = MsgBuffer[i + 1];
    }
    MsgBuffer[MSGMAXLEN - 1] = (unsigned char)SerialUSB.read();
    //SerialUSB.print((char)MsgBuffer[MSGMAXLEN-1]);
    ParseMsg(0);
  }
}

void ParseMsg(uint8_t interface)
{
  // Message JJAH: Hello Message (This is a presentation message when the API connect to the robot) => Enable WIFI output messages (if the message comes from a wifi interface)
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'H')) {
    SerialUSB.println("->MSG: JJAH: HELLO!");
    newMessage = 0; // No message to proccess on main code...
    working = false;
    trajectory_processing = false;
    if (interface == 1)
      enable_udp_output = true;
  }

  // Message JJAM: Manual control mode (Direct Kinematic)
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'M')) {
    SerialUSB.print("->MSG: JJAM:");
    iCH1 = ExtractParamInt2b(4);  // axis1
    iCH2 = ExtractParamInt2b(6);  // axis2
    iCH3 = ExtractParamInt2b(8);  // z
    iCH4 = ExtractParamInt2b(10); // servo1 => orientation
    iCH5 = ExtractParamInt2b(12); // servo2 => gripper
    iCH6 = ExtractParamInt2b(14);
    iCH7 = ExtractParamInt2b(16);
    iCH8 = ExtractParamInt2b(18);
    SerialUSB.print(iCH1);
    SerialUSB.print(" ");
    SerialUSB.print(iCH2);
    SerialUSB.print(" ");
    SerialUSB.println(iCH3);
    mode = 1;
    newMessage = 1;
    working = true; // Work to do...
    trajectory_processing = false;
    if (interface == 1)
      enable_udp_output = true;
  }
  // Message JJAI: Automatic control mode (Inverse Kinematic)
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'I')) {
    SerialUSB.println("->MSG: JJAI:");
    iCH1 = ExtractParamInt2b(4);  // target x
    iCH2 = ExtractParamInt2b(6);  // target y
    iCH3 = ExtractParamInt2b(8);  // target z
    iCH4 = ExtractParamInt2b(10); // target wrist orientation(servo1)servo1 => orientation
    iCH5 = ExtractParamInt2b(12); // target gripper position(servo2)
    iCH6 = ExtractParamInt2b(14); // solution type : 0 positive, 1 negative (elbow position)
    iCH7 = ExtractParamInt2b(16);
    iCH8 = ExtractParamInt2b(18);
    mode = 2;
    newMessage = 1;
    working = true; // Work to do...
    trajectory_processing = false;
    if (interface == 1)
      enable_udp_output = true;
  }
  // Message JJAT: Trajectory mode (Inverse Kinematic)
  // In this mode the robot draws a trajectory defined by a set of points. The robot don´t decelerate on intermediate points.
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'T')) {
    //SerialUSB.println("->MSG: JJAT:");
    trajectory_processing = true;   // trajectory proccesing mode...
    working = false;  
    iCH1 = ExtractParamInt2b(4);   //target Angle1
    iCH2 = ExtractParamInt2b(6);   //target Angle2
    iCH3 = ExtractParamInt2b(8);   //target z
    iCH4 = ExtractParamInt2b(10);  //target wrist orientation(servo1)servo1 => orientation
    iCH5 = ExtractParamInt2b(12);  //target gripper position(servo2)
    iCH6 = ExtractParamInt2b(14);  //solution type : 0 positive, 1 negative (elbow position)
    iCH7 = ExtractParamInt2b(16);  //Point number : from 0 to 50 (max points)
    iCH8 = ExtractParamInt2b(18);  //Last point: 0: intermediate point, 1:last point
    mode = 3; // Line Trajectory mode

    SerialUSB.print("-->JJAT ");
    SerialUSB.print(iCH7);
    SerialUSB.print(" :");
    SerialUSB.print(iCH1);
    SerialUSB.print(" ");
    SerialUSB.print(iCH2);
    SerialUSB.print(" ");
    SerialUSB.println(iCH3);
    
    if (iCH7 == 0) {
      // First point? => empty trajectory vector
      for (int i = 0; i < MAX_TRAJECTORY_POINTS; i++)
        for (int j = 0; j < 5; j++)
          trajectory_vector[i][j] = 0;
    }
    if ((iCH7 >= MAX_TRAJECTORY_POINTS)||(iCH7<0)) {
      SerialUSB.println("-->TR POINT OVERFLOW!");
      iCH7 = MAX_TRAJECTORY_POINTS - 1;
    }
    else {
      trajectory_vector[iCH7][0] = iCH1/100.0;
      trajectory_vector[iCH7][1] = iCH2/100.0;
      if (iCH3 == NODATA)
        trajectory_vector[iCH7][2] = NODATA;
      else
        trajectory_vector[iCH7][2] = iCH3/100.0;
      trajectory_vector[iCH7][3] = iCH4;
      trajectory_vector[iCH7][4] = iCH5;
    }
    if (iCH8 == 1) { // iCH8==1 means Last point=>execute
      trajectory_mode = true;  // Execute trajectory
      trajectory_num_points = iCH7;
      trajectory_point = 0;
      working = true;  // Work to do...
    }
    else
      trajectory_mode = false;

    newMessage = 1;
    if (interface == 1)
      enable_udp_output = true;
  }

  // Setup message "JJAS" Set robot speed and acc
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'S')) {
    SerialUSB.print("->MSG: JJAS:");
    iCH1 = ExtractParamInt2b(4);
    iCH2 = ExtractParamInt2b(6);
    iCH3 = ExtractParamInt2b(8);
    iCH4 = ExtractParamInt2b(10);
    iCH5 = ExtractParamInt2b(12);  // Trajectory speed (default=20) More speed->less accuracy
    SerialUSB.print(" SPEED XY:");
    SerialUSB.print(iCH1);
    //SerialUSB.print(" ");
    //SerialUSB.print((MAX_SPEED_M1 * float(iCH1)) / 100.0);
    SerialUSB.print(" SPEED Z:");
    SerialUSB.print(iCH2);
    //SerialUSB.print(" ");
    //SerialUSB.print((MAX_SPEED_M3 * float(iCH2)) / 100.0);
    SerialUSB.print(" ACC XY:");
    SerialUSB.print(iCH3);
    SerialUSB.print("ACC Z:");
    SerialUSB.print(iCH4);
    SerialUSB.print(" TRAJ S:");
    SerialUSB.print(iCH5);
    
    trajectory_tolerance_M1 = (iCH5/10.0f)*M1_AXIS_STEPS_PER_UNIT;
    trajectory_tolerance_M2 = (iCH5/10.0f)*M2_AXIS_STEPS_PER_UNIT;
    trajectory_tolerance_M3 = (iCH5/10.0f)*M3_AXIS_STEPS_PER_UNIT;

    SerialUSB.print(" ");
    SerialUSB.print(trajectory_tolerance_M1);
    SerialUSB.print(" ");
    SerialUSB.print(trajectory_tolerance_M2);
    SerialUSB.print(" ");
    SerialUSB.println(trajectory_tolerance_M3);

    configSpeed((MAX_SPEED_M1 * float(iCH1)) / 100.0, (MAX_SPEED_M2 * float(iCH1)) / 100.0, (MAX_SPEED_M3 * float(iCH2)) / 100.0);
    configAcceleration((MAX_ACCEL_M1 * float(iCH3)) / 100.0, (MAX_ACCEL_M2 * float(iCH3)) / 100.0, (MAX_ACCEL_M3 * float(iCH4)) / 100.0);
    if (interface == 1)
      enable_udp_output = true;
  }

  // robot motors calibration message "JJAC" 
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'C')) {
    SerialUSB.println("->MSG: JJAC:");
    working = 1;
    newMessage = 1;
    mode = 5;        // Calibration mode
    if (interface == 1)
      enable_udp_output = true;
  }

  // Emergency stop message "JJAE" Stops the robot and disble motors
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'E')) {
    SerialUSB.println("->MSG: JJAE:");
    working = 1;
    newMessage = 1;
    mode = 4;        // Emergency stop mode
    if (interface == 1)
      enable_udp_output = true;
  }
}


void debugMsg()
{
  SerialUSB.print("->mode:");
  SerialUSB.print(mode);
  SerialUSB.print(" CH:");
  SerialUSB.print(iCH1);
  SerialUSB.print(" ");
  SerialUSB.print(iCH2);
  SerialUSB.print(" ");
  SerialUSB.print(iCH3);
  SerialUSB.print(" ");
  SerialUSB.print(iCH4);
  SerialUSB.print(" ");
  SerialUSB.print(iCH5);
  SerialUSB.print(" ");
  SerialUSB.print(iCH6);
  SerialUSB.print(" ");
  SerialUSB.print(iCH7);
  SerialUSB.print(" ");
  SerialUSB.println(iCH8);
}
