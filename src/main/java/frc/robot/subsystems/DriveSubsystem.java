// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.CoralSubsystem;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.*;

import java.io.IOException;
import java.text.ParseException;

import com.ctre.phoenix6.StatusSignal;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.controller.PIDController;
import com.pathplanner.lib.config.PIDConstants;



public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);
  
  public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0), // Front Left
          new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0), // Front Right
          new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0), // Back Left
          new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0) // Back Right
      );
  // The gyro sensor
  //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  //pigeon gyro definition (Uncoment when using pidgeon)
  //PigeonIMU m_PigeonIMU = new PigeonIMU(Constants.OIConstants.kPigeonIMUPort);
  private final Pigeon2 pigeon = new Pigeon2(1, "rio");
  private final AHRS navX = new AHRS(NavXComType.kUSB1);

  private final PIDController xController = new PIDController(9.0, 0, 0);
  private final PIDController yController = new PIDController(5.0, 0, 0);
  private final PIDController headingController = new PIDController(0.03, 0, 0);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(pigeon.getRotation2d().getRadians()),
      //Rotation2d.fromDegrees(navX.getRotation2d().getRadians()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    PPHolonomicDriveController driveController = new PPHolonomicDriveController(new PIDConstants(7.0, 0.0, 0.0), new PIDConstants(7.0, 0.0, 0.0));
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException e) {
      e.printStackTrace();
      throw new RuntimeException("BAD SETTINGS FILE");
    } catch (org.json.simple.parser.ParseException e) {
      e.printStackTrace();
      throw new RuntimeException("PARSING ERROR");
    }

    
    

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      driveController, 
      config,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      this
    );

	headingController.enableContinuousInput(-Math.PI, Math.PI);

	// Usage reporting for MAXSwerve template
	HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    resetEncoders();
    
  }

  @Override
  public void periodic() {

  SmartDashboard.putNumber("Yaw", pigeon.getRotation2d().getRadians());
	SmartDashboard.putNumber("Pitch", pigeon.getPitch().getValueAsDouble());
	SmartDashboard.putNumber("Roll", pigeon.getRoll().getValueAsDouble());

  //SmartDashboard.putNumber("Yaw", navX.getRotation2d().getRadians());
  //SmartDashboard.putNumber("Pitch", navX.getPitch());
  //SmartDashboard.putNumber("Roll", navX.getRoll());

  

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(pigeon.getRotation2d().getRadians()),
        //Rotation2d.fromDegrees(navX.getRotation2d().getRadians()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(pigeon.getRotation2d().getRadians()),
        //Rotation2d.fromDegrees(navX.getRotation2d().getRadians()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    

	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	double rightTriggerValue = (m_driverController.getRightTriggerAxis() * -0.8) + 1.0;

	if(xSpeed < -0.01 && xSpeed > -0.1) {
		xSpeed = 0.0;
	}
	if(ySpeed < 0.01 && ySpeed > 0.01) {
		ySpeed = 0.0;
	}

	xSpeed = xSpeed * rightTriggerValue;
	ySpeed = ySpeed * rightTriggerValue;
	rot = rot * rightTriggerValue;

  SmartDashboard.putNumber("rot", (double)rot);

	double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(pigeon.getRotation2d().getRadians()))
                  //Rotation2d.fromRadians(navX.getRotation2d().getRadians()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon.reset();
    //navX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(pigeon.getRotation2d().getRadians()).getDegrees();
    //return navX.getRotation2d().getDegrees();
  }

  public ChassisSpeeds getSpeeds() {
    return kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()});
  }

  void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    var targetStates = kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
    

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //StatusSignal<AngularVelocity> vel =  pigeon.getAngularVelocityYWorld();
    return pigeon.getAngularVelocityYWorld().getValueAsDouble();
    //return navX.getVelocityY();
  }

//   public void followTrajectory(SwerveSample sample) {

//   }
}
