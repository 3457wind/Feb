#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 1;
  switch (auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;
    case 3:
      redGoalRush();
      break;
    case 4:
      left_4ball();
      break;
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      alliance_solo();
      break;
    case 9:
      break;
    case 10:
      break;
    case 11:
      break;
    case 12:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

void runDriver() {
  bool match_loader_state = false;
  bool midDS_state = false;
  bool descore_state = false;
  bool middle_piston_state = false;

  // button edge tracking (so toggle only happens once per press)
  bool last_y = false;
  bool last_b = false;
  bool last_a = false;

  stopChassis(coast);
  heading_correction = false;

  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // -----------------------------
    // TOGGLES (independent)
    // -----------------------------
    if (button_y && !last_y) {
      match_loader_state = !match_loader_state;
      match_loader.set(match_loader_state);
    }

    if (button_b && !last_b) {
      descore_state = !descore_state;
      descore.set(descore_state);
    }

    if (button_a && !last_a) {
      middle_piston_state = !middle_piston_state;
      middle_piston.set(middle_piston_state);
    }

    last_y = button_y;
    last_b = button_b;
    last_a = button_a;

    // -----------------------------
    // INTAKES (independent)
    // -----------------------------
    // lower intake on L1/L2
    if (l1) {
      lower_intake.spin(fwd, 100, pct);
    } else if (l2) {
      lower_intake.spin(reverse, 100, pct);
    } else {
      lower_intake.stop(coast);
    }

    // upper intake on R1/R2 (R1 slow if middle piston is true)
    if (r1 && middle_piston_state == true) {
      upper_intake.spin(fwd, 50, pct);
    } else if (r1) {
      upper_intake.spin(fwd, 100, pct);
    } else if (r2) {
      upper_intake.spin(reverse, 100, pct);
    } else {
      upper_intake.stop(coast);
    }

    // default tank drive or replace it with your preferred driver code here:
    // driveChassis(ch3 * 0.12, ch2 * 0.12); // tank mode
    driveChassis(ch3 * 0.06 + ch1 * 0.06, ch3 * 0.06 - ch1 * 0.06); // arcade mode

    wait(10, msec);
  }
}

void runPreAutonomous() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);

  // odom tracking
  resetChassis();
  if (using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
}

