// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;

public class PIDTurnCCW extends CommandBase {
  DriveTrain dt;
  double setPointAngle;
  PIDController PID = new PIDController(0.3/90, 0.005, 0);
  int motorSign;
  
  /** Creates a new PidTurnCCW. */
  public PIDTurnCCW(DriveTrain dt, double setPointAngle) {
    this.dt = dt;
    this.setPointAngle = setPointAngle; //sets setPointAngle to setPointAngle
    PID.setTolerance(10.0); //sets the tolerance (range of error)
    addRequirements(dt);
    if (setPointAngle >= 0) {
      motorSign = 1;//counterclockwise turn
    }
    else {
      motorSign = -1; //clockwise
    }
    //Clockwise turn
    }
    // Use addRequirements() here to declare subsystem dependencies.
  
//To get the P constant, motor power divided by set point

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavx();
    //sensor that gets the angle
    dt.tankDrive (0,0); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = PID.calculate(dt.getAngle(), setPointAngle);
    //calculates the output
    dt.tankDrive(-output*motorSign, output*motorSign);
    //has the left wheel turn backwards, right wheel turn forward
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0,0.0);
    //stops the robot
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PID.atSetpoint();
    //constantly checks the PID if it is at Setpoint. Once it is, the return overrides and ends the code
  }
}
