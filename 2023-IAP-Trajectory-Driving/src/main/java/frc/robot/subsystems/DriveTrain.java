// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final TalonSRXSimCollection leftDriveSim;
  private final TalonSRXSimCollection rightDriveSim;

  private AHRS navx = new AHRS(SPI.Port.kMXP); //This is a constructor, which creates an object in the AHRS class called navx

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain"); //This titles the variable DTTab into DriveTrain
  private double simLeftVoltage = 0; 
  private double simRightVoltage = 0;
  private DifferentialDrivetrainSim mDriveSim;
  private Field2d mField;
  private DifferentialDrive drive;
  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);//Creating an object by using a constructor for LeftDriveTalon
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort); //Same as above except for RightDriveTalon
    leftDriveSim = leftDriveTalon.getSimCollection();
    rightDriveSim = rightDriveTalon.getSimCollection();
    drive = new DifferentialDrive(rightDriveTalon, leftDriveTalon);


    leftDriveTalon.setNeutralMode(NeutralMode.Coast); //Sets leftDriveTalon to neutral
    rightDriveTalon.setNeutralMode(NeutralMode.Coast); //Sets rightDriveTalon to neutral

    leftDriveTalon.setInverted(false); //Make sure the leftDriveTalon is inverted compared to the rightDriveTalon so it can move forward and backward correctly
    rightDriveTalon.setInverted(true);  //Make sure the rightDriveTalon is inverted compared to the leftDriveTalon so it can move forward and backward correctly

    leftDriveTalon.setSensorPhase(true); //Set the sensors for leftDriveTalon to true
    rightDriveTalon.setSensorPhase(true); //Set the sensors for rightDriveTalon to true

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //Set up encoder to track rotation of motor
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // Create the simulation model of our drivetrain.
    mDriveSim = new DifferentialDrivetrainSim(
  DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
  7.29,                    // 7.29:1 gearing reduction.
  7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  60.0,                    // The mass of the robot is 60 kg.
  Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  0.7112,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  mField = new Field2d();
  mField.setRobotPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void tankDrive(double leftSpeed, double rightSpeed) { //This will drive robot with certain speed
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
    simLeftVoltage = leftSpeed*12.0;
    simRightVoltage = rightSpeed*12.0;
    drive.feed();
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10); //Sets sensor position of leftDriveTalon to 0,0,10
    rightDriveTalon.setSelectedSensorPosition(0,0,10); //Sets sensor position of rightDriveTalon to 0,0,10
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }
  public double getMeters(){
    return (0.1524 * Math.PI / 4096) * getTicks();
  }
  public double getAngle(){ //Gets the robot's current angle
    return navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());
  }
  @Override
  public void simulationPeriodic() {
    mDriveSim.setInputs(simLeftVoltage, simRightVoltage);
    mDriveSim.update (0.02);
    mField.setRobotPose(mDriveSim.getPose());
    SmartDashboard.putData("Field", mField);
  }
}