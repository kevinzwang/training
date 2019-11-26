/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Encoder;


public class AutonomousForward extends Command {
  private Encoder leftEnc, rightEnc;
  private Drivetrain motors;

  public AutonomousForward(Encoder leftEnc, Encoder rightEnc,Drivetrain motors) {
    this.leftEnc = leftEnc;
    this.leftEnc = rightEnc;
    this.motors = motors;

    double pulseFraction = 1.0 / 256;
    double wheelDiameter = 5;
    leftEnc.setDistancePerPulse(pulseFraction * Math.PI * wheelDiameter);
    rightEnc.setDistancePerPulse(pulseFraction * Math.PI * wheelDiameter);

    rightEnc.setReverseDirection(true);
    requires(motors);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

  }
  

  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftEnc.reset();
    rightEnc.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      motors.arcaderun(0.5,0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return leftEnc.get() > 24 && rightEnc.get() > 24;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  
}
