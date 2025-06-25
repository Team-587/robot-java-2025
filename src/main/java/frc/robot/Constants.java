// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final int kLEDLength = 69;

    // Chassis configuration
    public static final double kTrackWidth = 0.5468;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5468;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 18;
    public static final int kRearLeftDrivingCanId = 16;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 17;
    public static final int kRearLeftTurningCanId = 15;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;

    public static final int kUppies1CanId = 22;
    public static final int kUppies2CanId = 23;
    public static final int kHouseWristCanId = 24;
    public static final int kHouseCanId = 26;

    public static final int kHouseSwitch = 1;
    public static final int kUppiesSwitch = 2;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final double kDrivingMotorPinionTeeth1 = 14.0;
    public static final double kDrivingMotorPinionTeeth2 = 28.0;
    public static final double kDrivingMotorPinionTeeth3 = 15.0;
    public static final double kDrivenMotorPinionTeeth1 = 50.0;
    public static final double kDrivenMotorPinionTeeth2 = 16.0;
    public static final double kDrivenMotorPinionTeeth3 = 45.0;
    
    

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60; //NEO Free speed is 5820 rpm, Vortex free speed is 6784 rpm as of 11/18/2024
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (kDrivenMotorPinionTeeth1 / kDrivingMotorPinionTeeth1) 
				* (kDrivenMotorPinionTeeth2 / kDrivingMotorPinionTeeth2) 
				* (kDrivenMotorPinionTeeth3 / kDrivingMotorPinionTeeth3);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
	  public static final int kCoDriverControllerPort = 1;
    public static final int kPigeonIMUPort = 2;
    public static final double kDriveDeadband = 0.05;
    public static final int kLEDPort = 3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.9;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VortexMotorConstants {
	public static final double kFreeSpeedRpm = 6784;
  }
}
