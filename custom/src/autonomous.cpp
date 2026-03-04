#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

// Ryan Auto

void left_4ball() // case 4
{
 driveTo(30,2750,true,9);
  wait(250, msec);
  match_loader.set(true);
  descore.set(true);
  turnToAngle(90,1000,true,6);
  wait(100, msec);
  lower_intake.spin(fwd,90, pct);
  driveTo(19,1300,true,7);
  wait(1000,msec);
  driveTo(-42,1750,true,4);
  match_loader.set(false);
  driveTo(18,1100,true,6);
  turnToAngle(45,1000,true,6);
  driveTo(8*sqrt(2),1000,true,6);
  wait(500,msec);
  turnToAngle(90,1000,true,6);
  driveTo(-112,2000,true,6);
  turnToAngle(-45,1000,true,6);
  driveTo(-8*sqrt(2),1000,true,6);
  turnToAngle(-90,1000,true,6);
  driveTo(-19,1500,true,6);

}

void alliance_solo() // case 8
{
  lower_intake.spin(fwd,90, pct);
  driveTo(54,3000,true,9);
  wait(200, msec);
  turnToAngle(90,1000,true,6);
  match_loader.set(true);
  wait(200, msec);
  driveTo(18,15000,true,6);
  wait(900,msec);
  driveTo(-43,1750,true,4);
  wait(300,msec);
  upper_intake.spin(fwd,100,pct);
  wait(1200,msec);
  upper_intake.stop(coast);
  match_loader.set(false);
  driveTo(24,1500,true,6);
  turnToAngle(225,1250,true,6);
  driveTo(36,1000,true,9);
  wait(500,msec);
  turnToAngle(180,750,true,9);
  driveTo(54,2750,true,9);
  turnToAngle(135,500,true,12);
  
}


void leftAuton() {
  driveTo(30,2750,true,9);
  wait(250, msec);
  match_loader.set(true);
  turnToAngle(90,1000,true,6);
  wait(100, msec);
  lower_intake.spin(fwd,90, pct);
  driveTo(19,1300,true,7);
  wait(300,msec);
  driveTo(-42,1750,true,4);
  wait(250,msec);
  upper_intake.spin(fwd,100,pct);
  wait(1000,msec);
  upper_intake.stop(coast);
  match_loader.set(false);
  driveTo(18,1100,true,6);
  turnToAngle(225,1000,true,6);
  driveTo(36,1000,true,6);
  wait(500,msec);
  lower_intake.stop(coast);
  driveTo(33,1000,true,6);
  lower_intake.spin(reverse,25,pct);
}

void exampleAuton2() {
   lower_intake.spin(fwd,90, pct);
  driveTo(46,3000,true,9);
  wait(200, msec);
  turnToAngle(90,1000,true,6);
  match_loader.set(true);
  wait(200, msec);
  driveTo(18,15000,true,6);
  wait(700,msec);
  driveTo(-43,1800,true,5);
  wait(300,msec);
  upper_intake.spin(fwd,100,pct);
  wait(1200,msec);
  match_loader.set(false);
  upper_intake.stop(coast);
  driveTo(26,1500,true,6);
  turnToAngle(225,1250,true,6);
  driveTo(36,1000,true,9);
  wait(2000,msec);
  turnToAngle(45,750,true,9);
  driveTo(36,1000,true,9);
  turnToAngle(90,750,true,9);
  driveTo(-26,1500,true,6);
  upper_intake.spin(fwd,100,pct);
}

double arm_pid_target = 0, arm_load_target = 60, arm_store_target = 250, arm_score_target = 470;

/*
 * armPID
 * Runs a single PID update for the arm motor to reach the specified target position.
 * - arm_target: Desired arm position (degrees).
 */
void armPID(double arm_target) {
  PID pidarm = PID(0.1, 0, 0.5); // Initialize PID controller for arm
  pidarm.setTarget(arm_target);   // Set target position
  pidarm.setIntegralMax(0);  
  pidarm.setIntegralRange(1);
  pidarm.setSmallBigErrorTolerance(1, 1);
  pidarm.setSmallBigErrorDuration(0, 0);
  pidarm.setDerivativeTolerance(100);
  pidarm.setArrive(true);
  arm_motor.spin(fwd, pidarm.update(arm_motor.position(deg)), volt); // Apply PID output to arm motor
}

/*
 * armPIDLoop
 * Continuously runs the arm PID control in a separate thread, keeping the arm at the target position.
 */
void armPIDLoop() {
  while(true) {
    armPID(arm_pid_target); // Continuously update arm position
    wait(10, msec);
  }
}

/*
 * rushClamp
 * Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
 * Used for quickly grabbing a mobile goal at the start of autonomous.
 */
void rushClamp() {
  while(clamp_distance.objectDistance(mm) > 85) { // Wait for object to be close enough
    wait(10, msec);
  }
  claw.set(true);        // Close the claw to grab the goal
  rush_arm.set(false);   // Lower the rush arm
}

/*
 * intakeThread
 * Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
 * Used for picking up rings or other objects during autonomous.
 */
void intakeThread(){
  optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
  optical_sensor.setLightPower(100);          // Set light power to max
  while(!optical_sensor.isNearObject() && intake_distance.objectDistance(mm) > 50){
    wait(10, msec);                           // Wait until object is detected
  }
  intake_motor.stop(hold);                    // Stop intake motor and hold
}

/*
 * redGoalRush
 * 2024-2025 World Championship runner-up(1698V) autonomous routine.
 * This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
 * It uses multiple threads for simultaneous arm, clamp, and intake control.
 */
void redGoalRush() {
  arm_motor.setPosition(arm_load_target, deg);         // Set arm to load position
  correct_angle = inertial_sensor.rotation();          // Sync correct_angle with inertial sensor
  arm_pid_target = arm_store_target;                   // Set arm PID target to store position

  thread al = thread(armPIDLoop);                      // Start arm PID loop in a thread
  thread rc = thread(rushClamp);                       // Start clamp routine in a thread
  intake_motor.spin(fwd, 12, volt);                    // Start intake motor at full speed
  thread it = thread(intakeThread);                    // Start intake sensor thread
  rush_arm.set(true);                                  // Lower rush arm

  driveTo(33, 1100, true);                             // Drive forward to first goal
  moveToPoint(-2, 10, -1, 15000, false);               // Pull the goal back
  stopChassis(hold);                                   // Stop chassis and hold position

  rc.interrupt();                                      // Stop clamp thread (goal should be clamped)
  rush_arm.set(true);                                  // Lower rush arm again (ensure down)
  claw.set(false);                                     // Open claw to release goal
  wait(100, msec);                                     // Brief pause

  correct_angle = normalizeTarget(-20);                // Adjust target heading for next maneuver
  driveTo(3, 800, true, 8);                            // Drive forward slightly
  driveTo(-5, 1000, true);                             // Back up

  rush_arm.set(false);                                 // Raise rush arm
  wait(200, msec);                                     // Wait for arm to raise

  turnToAngle(-90, 800, false);                        // Turn to face the goal backwards
  moveToPoint(0, 26, -1, 2000, false, 6);              // Move backwards into the goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  it.interrupt();                                      // Stop intake thread (ring should be collected)
  intake_motor.spin(fwd, 12, volt);                    // Restart intake

  moveToPoint(1, 7, 1, 2000, true);                    // Move near corner to drop goal
  turnToAngle(-90, 350, true);                         // Turn to drop goal
  mogo_mech.set(false);                                // Release mobile goal clamp
  driveChassis(-4, 4);                                 // Turn a bit to align with next target
  wait(300, msec);                                     // Wait for spin

  intake_motor.spin(fwd, -12, volt);                   // Reverse intake to push disc in front away
  moveToPoint(-13, -4, 1, 1500, false, 10);            // Move forward to push disc out of the way
  turnToAngle(180, 800, false);                        // Turn to clamp goal
  intake_motor.spin(fwd, 0, volt);                     // Stop intake

  moveToPoint(-31, 26, -1, 2000, false, 6);            // Move backwards into the next goal
  driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
  mogo_mech.set(true);                                 // Clamp mobile goal
  wait(100, msec);                                     // Wait for clamp

  turnToAngle(145, 300, true);                         // Turn to face corner
  moveToPoint(-4, -3, 1, 2000, false);                 // Move to corner
  intake_motor.spin(fwd, 12, volt);                    // Start intake

  correct_angle = normalizeTarget(135);                // Update heading for next maneuver
  driveTo(1000, 1500, false, 4);                       // Drive forward infinitely until timeout
  driveTo(-13, 2000, true, 6);                         // Back up
  driveTo(10, 2500, true, 3);                          // Drive forward to intake second corner ring

  wait(200, msec);                                     // Brief wait for intake

  moveToPoint(-11, 6, -1, 2000, false, 10);            // Move backward out of the corner
  turnToAngle(45, 400, true);                          // Turn to align for wallstake

  al.interrupt();                                      // Stop arm PID thread
  arm_pid_target = arm_score_target - 100;             // Set arm to scoring position
  thread al2 = thread(armPIDLoop);                     // Start new arm PID thread

  moveToPoint(12, 34, 1, 1700, true, 8);               // Move forward to final wallstake scoring position

  al2.interrupt();                                     // Stop arm PID thread
  arm_motor.spin(fwd, 1, volt);                        // Spin arm forward slightly

  turnToAngle(40, 200);                                // Final turn for alignment
  driveChassis(1, 1);                                  // Slow drive forward
}