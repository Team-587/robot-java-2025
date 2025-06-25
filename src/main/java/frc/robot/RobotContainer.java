// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralSubsystem.coralState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.subsystems.AutoAlignLeft;
import frc.robot.subsystems.AutoAlignRight;
import frc.robot.subsystems.CoralSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase {
  // The robot's subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final AutoAlignLeft m_autoAlignLeft = new AutoAlignLeft(m_drive);
  private final AutoAlignRight m_autoAlignRight = new AutoAlignRight(m_drive);
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);


  public Command zeroHeading() {
    return this.runOnce(() -> m_drive.zeroHeading());
  }
  public Command setX(){
    return this.run(() -> m_drive.setX());
  }
  public Command m_coralClimb(){
    return this.runOnce(() -> m_coralSubsystem.climbMode());
  }
  public Command m_stow(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.STOW));
  }
  public Command m_level1(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.LEVEL1));
  }
  public Command m_level2(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.LEVEL2));
  }
  public Command m_level3(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.LEVEL3));
  }
  public Command m_level4(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.LEVEL4));
  }
  public Command m_algaeLow(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.ALGAE1));
  }
  public Command m_algaeHigh(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.ALGAE2));
  }
  public Command m_algaeScore(){
    return this.runOnce(() -> m_coralSubsystem.setState(coralState.ALGAESCORE));
  }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    

    // Configure default commands
    m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_drive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_drive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
     /*new JoystickButton(m_driverController, Button.kR1.value)
         .whileTrue(new RunCommand(
             () -> m_drive.setX(),
             m_drive));*/

    new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(zeroHeading());
    new JoystickButton(m_driverController, XboxController.Button.kX.value).whileTrue(setX());
    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value).whileTrue(m_autoAlignRight);
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value).whileTrue(m_autoAlignLeft);
    new JoystickButton(m_coDriverController, XboxController.Button.kX.value).onTrue(m_level1());
    new JoystickButton(m_coDriverController, XboxController.Button.kY.value).onTrue(m_level2());
    new JoystickButton(m_coDriverController, XboxController.Button.kB.value).onTrue(m_level3());
    new JoystickButton(m_coDriverController, XboxController.Button.kA.value).onTrue(m_level4());
    new JoystickButton(m_coDriverController, XboxController.Button.kLeftStick.value).onTrue(m_algaeLow());
    new JoystickButton(m_coDriverController, XboxController.Button.kRightStick.value).onTrue(m_algaeHigh());
    new JoystickButton(m_coDriverController, XboxController.Button.kRightBumper.value).onTrue(m_algaeScore());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drive::setModuleStates,
        m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, false));
  }
}
