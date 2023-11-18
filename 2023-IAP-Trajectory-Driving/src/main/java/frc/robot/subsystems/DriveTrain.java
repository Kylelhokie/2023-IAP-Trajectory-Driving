// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import javax.naming.spi.NamingManager;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final TalonSRXSimCollection leftDriveSim;
  private final TalonSRXSimCollection rightDriveSim;
  private final DifferentialDriveOdometry odometry;
 
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
        // Create a linear system from our identification gains.
        LinearSystemId.identifyDrivetrainSystem(Constants.SimConstants.kv,
        Constants.SimConstants.kA, Constants.SimConstants.kVangular,
        Constants.SimConstants.kAangular),
        DCMotor.getCIM(1), // 1 CIM motor on each side of the drivetrain.
        10.71, // 10.71:1 gearing reduction.
        Constants.SimConstants.kTrackwidthMeters, // The track width is 0.7112
        Units.inchesToMeters(3), // The robot uses 3" radius wheels.
          
        // The standard deviations for measurement noise:
        // x and y: 0.001 m
        // heading: 0.001 rad
        // l and r velocity: 0.1 m/s
        // l and r position: 0.005 m
        VecBuilder.fill(0.0001, 0.0001, 0.0001, 0.01, 0.01, 0.0005, 0.0005)
      );
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
  mDriveSim.setPose(new Pose2d(7, 4, new Rotation2d(0)));
  mField = new Field2d();
  odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());
  mField.setRobotPose(new Pose2d(7, 4, new Rotation2d(0)));

  resetEncoders();
  navx.reset();
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
  public double MeterstoTicks(double positionMeters) {
    return (positionMeters / (0.1524 * Math.PI)*4096);
  }
  public double getMeters(){
    return (0.1524 * Math.PI / 4096) * getTicks();
  }
  public double getAngle(){ //Gets the robot's current angle
    return -navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  public double getLeftDistance() {
    return leftDriveTalon.getSelectedSensorPosition() / Constants.DriveToLineConstants.ticksToMeters;
  }

  /**
   * Returns displacement of right side of chassis.
   * 
   * @return the displacement in meters (m)
   */
  public double getRightDistance() {

    return rightDriveTalon.getSelectedSensorPosition() / Constants.DriveToLineConstants.ticksToMeters;
  }
  public double getLeftSpeed() {
    return (leftDriveTalon.getSelectedSensorVelocity() * 10.0) / Constants.DriveToLineConstants.ticksToMeters;
  }
  public double getRightSpeed() {
    return (leftDriveTalon.getSelectedSensorVelocity() * 10.0) / Constants.DriveToLineConstants.ticksToMeters;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());


    
    mDriveSim.setInputs(simLeftVoltage, simRightVoltage);
    mDriveSim.update (0.02);
    mField.setRobotPose(mDriveSim.getPose());
    SmartDashboard.putData("Field", mField);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(mDriveSim.getHeading().getDegrees());
    leftDriveSim.setQuadratureRawPosition(
      distanceToNativeUnits(
          -mDriveSim.getLeftPositionMeters()));
  leftDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(
          -mDriveSim.getLeftVelocityMetersPerSecond()));
  leftDriveTalon.setSelectedSensorPosition(MeterstoTicks(mDriveSim.getLeftPositionMeters()), 0, 10);
  rightDriveTalon.setSelectedSensorPosition(MeterstoTicks(mDriveSim.getRightPositionMeters()), 0,10);


  // Update Quadrature for Right
  // Have to flip, to match phase of real encoder
  // Left wheel goes CCW, Right goes CW for forward by default

  rightDriveSim.setQuadratureRawPosition( //sets the r
      distanceToNativeUnits(
          mDriveSim.getRightPositionMeters()));
  rightDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(
          mDriveSim.getRightVelocityMetersPerSecond())); 
          SmartDashboard.putNumber("Heading", mDriveSim.getHeading().getDegrees());

          SmartDashboard.putNumber("LeftPosition", getLeftDistance());
          SmartDashboard.putNumber("RightPosition", getRightDistance());
          SmartDashboard.putNumber("LeftVel", getLeftSpeed());
          SmartDashboard.putNumber("RightVel", getRightSpeed());
      
          // Turn rate returns 0 in sim, same in real life?
          // Turn rate is never used
          SmartDashboard.putNumber("TurnRate", getTurnRate());
          SmartDashboard.putNumber("SimAng", angle.get());
          if (Robot.isSimulation()) {
            odometry.update(navx.getRotation2d().unaryMinus(), getLeftDistance(), getRightDistance());
          }
      
        }
        public Pose2d getPose() {
          return odometry.getPoseMeters();
        }
    
  @Override
  public void simulationPeriodic() {
    mDriveSim.setInputs(simLeftVoltage, simRightVoltage);
    mDriveSim.update (0.02);
    mField.setRobotPose(mDriveSim.getPose());
    SmartDashboard.putData("Field", mField);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(mDriveSim.getHeading().getDegrees());
    leftDriveSim.setQuadratureRawPosition(
      distanceToNativeUnits(
          -mDriveSim.getLeftPositionMeters()));
  leftDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(
          -mDriveSim.getLeftVelocityMetersPerSecond()));
  leftDriveTalon.setSelectedSensorPosition(MeterstoTicks(mDriveSim.getLeftPositionMeters()), 0, 10);
  rightDriveTalon.setSelectedSensorPosition(MeterstoTicks(mDriveSim.getRightPositionMeters()), 0,10);


  // Update Quadrature for Right
  // Have to flip, to match phase of real encoder
  // Left wheel goes CCW, Right goes CW for forward by default

  rightDriveSim.setQuadratureRawPosition( //sets the r
      distanceToNativeUnits(
          mDriveSim.getRightPositionMeters()));
  rightDriveSim.setQuadratureVelocity(
      velocityToNativeUnits(
          mDriveSim.getRightVelocityMetersPerSecond()));
  }
  private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters
        / (Math.PI * Units.inchesToMeters(Constants.DriveTrainPorts.getMeters));
    double motorRotations = wheelRotations * 1.0;
    int sensorCounts = (int) (motorRotations * 4096.0);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond) {
    // Previous mistake: multiply this by 2
    // Consequences: had to set the constant to 0.5 less
    // Now it works without the 2
    double wheelRotationsPerSecond = velocityMetersPerSecond
        / (Math.PI * Units.inchesToMeters(Constants.DriveTrainPorts.getMeters));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 1.0; //Sets them equal to each other because when you multiply by 1.0, wheelRotationsPerSecond turns into a double
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0; //Sets them equal to each other because when you multiply by 10.0, wheelRotationsPerSecond turns into a double
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096.0); //
    return sensorCountsPer100ms;
  }

private double nativeUnitsToDistanceMeters(double sensorCounts) {
  double motorRotations = (double) sensorCounts / 4096.0;
  double wheelRotations = motorRotations / 1.0;
  double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(6.0));
  return positionMeters;
}
public double getAverageEncoderDistance() {
  return (getLeftDistance() + getRightDistance()) / 2.0;
}
public double getTurnRate() {
  return -navx.getRate();
}
}