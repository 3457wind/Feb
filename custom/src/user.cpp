#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include <cmath>
#include <algorithm>

// -----------------------------
// Expo + deadband helpers
// -----------------------------

// optional deadband for joystick drift
int applyDeadband(int v, int db) //function defined w output int, and input of int v and db
{
  return (std::abs(v) < db) ? 0 : v; //if return(stuff) is true - return 0, else return v
}

// input: -100..100  output: -100..100
double expoStick(double x, double expo)//function defined with output deci and inputs decis x and expo
{
  double sign = (x < 0) ? -1.0 : 1.0;//output deci, if x<0, output -1, else 1 
  double ax = std::fabs(x) / 100.0; //out deci
  // blend linear + cubic
  double shaped = (1.0 - expo) * ax + expo * (ax * ax * ax ); // linear shallows the cubic curve. if squared, faster mid range, but cubic is softer mid range. 
  return sign * shaped * 100.0; // retun the right sign, mixed output and 100 scale
}

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 2;
  switch (auton_selected) {
    case 1:
      leftAuton();//rightslot3
      break;
    case 2:
      exampleAuton2();//left
      break;
    case 3:
      redGoalRush();
      break;
    case 4:
      left_4ball();
      break;
    case 5:
      none();
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

    // EXPO ARCADE DRIVE
  int fwd_raw  = ch3;
int turn_raw = ch1;

// tune these
double turn_start = 5;    // start blending here
double turn_full  = 25;   // fully in slow mode here

double absTurn = std::fabs((double)turn_raw);

// t goes 0->1 as absTurn goes turn_start->turn_full
double t = (absTurn - turn_start) / (turn_full - turn_start);
t = std::max(0.0, std::min(1.0, t));

// optional: smoothstep so it ramps gently (less “kink”)
t = t * t * (3.0 - 2.0 * t);

// your caps
double fast_fwd_cap  = 0.225;
double fast_turn_cap = 0.225;

double slow_cap = 0.075;        // 1:1 in slow mode
double slow_fwd_cap  = slow_cap;
double slow_turn_cap = slow_cap;

// lerp caps
double fwd_cap  = fast_fwd_cap  + t * (slow_fwd_cap  - fast_fwd_cap);
double turn_cap = fast_turn_cap + t * (slow_turn_cap - fast_turn_cap);

double fwd  = expoStick((double)fwd_raw,  0.0);
double turn = expoStick((double)turn_raw, 0.0);

double left  = ((fwd * fwd_cap) + (turn * turn_cap));
double right = ((fwd * fwd_cap) - (turn * turn_cap));

left  = std::max(-100.0, std::min(100.0, left));
right = std::max(-100.0, std::min(100.0, right));

driveChassis(left, right);
}
}
void runPreAutonomous()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
wait(10,msec);
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