// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.LimelightHelpers;
import edu.wpi.first.units.VelocityUnit;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignRight extends Command {
  /** Creates a new AutoAlign. */
  double DesiredX = -0.46;
  double DesiredY = 0.19;
  double DesiredRot = -2.5;
  
  boolean readyToExit = false;

  PIDController m_xController;
  PIDController m_yController;
  PIDController m_rotController;
  
  DriveSubsystem m_drivebase;
  NetworkTable table;

  public AutoAlignRight(DriveSubsystem drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xController = new PIDController(0.4, 0.0, 0.0);
    m_yController = new PIDController(0.8, 0.0, 0.0);
    m_rotController = new PIDController(0.008, 0.0, 0.0);
    this.m_drivebase = drivebase;
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = NetworkTableInstance.getDefault().getTable("limelight-left");
    NetworkTableEntry numberEntry = table.getEntry("tid");
    double id = numberEntry.getDouble(0.0);
    
    if((id >= 6 && id <= 11) || (id >= 17 && id <= 22)){
      readyToExit = false;
    }else{
      readyToExit = true;
    }

    m_rotController.setSetpoint(DesiredRot);
    m_rotController.setTolerance(0.05);

    m_xController.setSetpoint(DesiredX);
    m_xController.setSetpoint(0.05);

    m_yController.setSetpoint(DesiredY);
    m_yController.setTolerance(0.05);

    SmartDashboard.putNumber("ID", id);
    SmartDashboard.putNumber("Desired X", DesiredX);
    SmartDashboard.putNumber("Desired Y", DesiredY);
    SmartDashboard.putNumber("Desired Rot", DesiredRot);

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableEntry numberEntry = table.getEntry("tid");
    double id = numberEntry.getDouble(0.0);

    double[] positions;
    positions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
    SmartDashboard.putNumber("Y", positions[0]);
    SmartDashboard.putNumber("X", positions[2]);
    if((id >= 6 && id <= 11) || (id >= 17 && id <= 22)){
      if(positions[2] > -1.0){
        m_xController.setP(0.6);
        m_yController.setP(1.0);
        m_rotController.setP(0.012);
      }else{
        m_xController.setP(0.7);
        m_yController.setP(1.0);
        m_rotController.setP(0.009);
      }
      double xSpeed = m_xController.calculate(positions[2]);
      double ySpeed = -m_yController.calculate(positions[0]);
      double rotValue = -m_rotController.calculate(positions[4]);
  
      m_drivebase.drive(xSpeed, ySpeed, rotValue, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.drive(0.0, 0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(readyToExit){
      return true;
    }
    double[] positions;
    positions = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
    double positionXError = positions[2] - DesiredX;
    double positionYError = positions[0] - DesiredY;
    double positionRotError = positions[4] - DesiredRot;
  
    if((positionXError < 0.1 && positionXError > -0.1) && 
          (positionYError < 0.1 && positionYError > -0.1) && 
          (positionRotError < 1.0 && positionRotError > -1.0)){
      return true;
    }else{
      return false;
    }
  }
}