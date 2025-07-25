package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)         // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.02, 0, 0)
                    .velocityFF(drivingVelocityFeedForward * 1.5)
                    .pid(0.08, 0, 0, ClosedLoopSlot.kSlot1)
                    .velocityFF(drivingVelocityFeedForward, ClosedLoopSlot.kSlot1)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .inverted(true);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(false)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
    public static final class CoralSubsystem {
        public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
        public static final SparkMaxConfig uppiesConfig1 = new SparkMaxConfig();
        public static final SparkMaxConfig uppiesConfig2 = new SparkMaxConfig();

        static {

            double turningFactorDegrees = 360;
            double turningFactorElevator = 1.432 * Math.PI;

            wristConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60)
                    .inverted(false);
            wristConfig
                    .absoluteEncoder
                    .inverted(false)
                    .positionConversionFactor(turningFactorDegrees)
                    .velocityConversionFactor(turningFactorDegrees / 60.0);
            wristConfig
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(0.00350, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(false);
                
            uppiesConfig1
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            uppiesConfig1
                    .alternateEncoder
                    .countsPerRevolution(8192)
                    .inverted(true)
                    .positionConversionFactor(turningFactorElevator)
                    .velocityConversionFactor(turningFactorElevator / 60);
            uppiesConfig1
                    .closedLoop
                    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                    .pid(0.45, 0, 0)
                    .outputRange(-0.7, 1);
            
            uppiesConfig2
                    .idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(50);
            uppiesConfig2
                    .follow(DriveConstants.kUppies1CanId, true);
        }
    }
}
