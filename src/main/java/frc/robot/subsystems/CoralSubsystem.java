// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

import com.fasterxml.jackson.core.util.Separators;
import com.revrobotics.AnalogInput;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  public enum coralState {
    STOW,
    LEVEL1,
    LEVEL2,
    LEVEL3,
    LEVEL4,
    ALGAE1,
    ALGAE2,
    ALGAESCORE
   };
  private final SparkFlex m_houseMotor;

  private final SparkFlex m_wristMotor;
  private final SparkAbsoluteEncoder m_wristEncoder;
  private final SparkClosedLoopController m_wristClosedLoopController;

  private final SparkMax m_uppiesMotor1;
  private final SparkMax m_uppiesMotor2;
  private final SparkMaxAlternateEncoder m_uppiesEncoder1;
  private final SparkClosedLoopController m_uppies1ClosedLoopController;

  private final DigitalInput m_houseSwitch;
  private final DigitalInput m_uppiesSwitch;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  boolean haveCoral;
  boolean climbActivated;
  double autoSpeed;
  double shootSpeed;
  coralState desiredState;

  public CoralSubsystem() {
    m_houseMotor = new SparkFlex(DriveConstants.kHouseCanId, MotorType.kBrushless);
    m_wristMotor = new SparkFlex(DriveConstants.kHouseWristCanId, MotorType.kBrushless);
    m_wristEncoder = m_wristMotor.getAbsoluteEncoder();
    m_wristClosedLoopController = m_wristMotor.getClosedLoopController();
    m_uppiesMotor1 = new SparkMax(DriveConstants.kUppies1CanId, MotorType.kBrushless);
    m_uppiesMotor2 = new SparkMax(DriveConstants.kUppies2CanId, MotorType.kBrushless);
    m_uppiesEncoder1 = (SparkMaxAlternateEncoder)m_uppiesMotor1.getAlternateEncoder();
    m_uppies1ClosedLoopController = m_uppiesMotor1.getClosedLoopController();
    m_houseSwitch = new DigitalInput(DriveConstants.kHouseSwitch);
    m_uppiesSwitch = new DigitalInput(DriveConstants.kUppiesSwitch);

    shootSpeed = 0.0;
    desiredState = coralState.STOW;

    m_wristMotor.configure(Configs.CoralSubsystem.wristConfig, 
                           SparkBase.ResetMode.kResetSafeParameters, 
                           SparkBase.PersistMode.kPersistParameters);

    m_uppiesMotor1.configure(Configs.CoralSubsystem.uppiesConfig1, 
                             SparkBase.ResetMode.kResetSafeParameters, 
                             SparkBase.PersistMode.kPersistParameters);
                             
    m_uppiesMotor2.configure(Configs.CoralSubsystem.uppiesConfig2, 
                             SparkBase.ResetMode.kResetSafeParameters, 
                             SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    haveCoral = !m_houseSwitch.get();
    double coDriverRT = m_coDriverController.getRightTriggerAxis();
    double coDriverLT = m_coDriverController.getLeftTriggerAxis();

    if(climbActivated){
      m_houseMotor.set(0.0);
      m_wristClosedLoopController.setReference(CoralConstants.kWristClimbAngle, ControlType.kPosition);
      m_uppies1ClosedLoopController.setReference(CoralConstants.kUppiesClimbHeight, ControlType.kPosition);
    }

    if(DriverStation.isAutonomousEnabled()){
      if(haveCoral == true && autoSpeed >= 0.0){
        m_houseMotor.set(CoralConstants.kBackspinAuto);
      }else{
        m_houseMotor.set(autoSpeed);
      }
    }else if(coDriverLT > 0.3 && haveCoral == false){
      m_houseMotor.set(CoralConstants.kHouseIntakeSpeed);
    }else if(coDriverRT > 0.3){
      m_houseMotor.set(shootSpeed);
    }else{
      m_houseMotor.set(CoralConstants.kBackspin);
    }

    switch (desiredState){
      //STOW
      case STOW:
        shootSpeed = CoralConstants.kHouseStowSpeed;
        m_houseMotor.set(CoralConstants.kBackspin);
        if(haveCoral == true){
          m_wristClosedLoopController.setReference(CoralConstants.kWristMoveAngle, ControlType.kPosition);
          if(checkWristAngle(CoralConstants.kWristMoveAngle)){
            m_uppies1ClosedLoopController.setReference(CoralConstants.kUppiesIntakeHeight, ControlType.kPosition);
          }
        }
        break;

      //LEVEL 1
      case LEVEL1:
        shootSpeed = CoralConstants.kHouseL1Speed;
        if(checkWristAngle(CoralConstants.kWristMoveAngle)){
          m_uppies1ClosedLoopController.setReference(CoralConstants.kUppiesL1Height, ControlType.kPosition);
        }
        if(checkUppiesHeight(CoralConstants.kUppiesL1Height)){
          if(DriverStation.isAutonomousEnabled()){
            m_wristClosedLoopController.setReference(CoralConstants.kWristL1AngleAuto, ControlType.kPosition);
          }else{
            m_wristClosedLoopController.setReference(CoralConstants.kWristL1Angle, ControlType.kPosition);
          }
        }else{
          m_wristClosedLoopController.setReference(CoralConstants.kWristMoveAngle, ControlType.kPosition);
        }
        break;

      //LEVEL 2
      case LEVEL2:
        shootSpeed = CoralConstants.kHouseL2Speed;
        if(checkWristAngle(CoralConstants.kWristMoveAngle)){
          m_uppies1ClosedLoopController.setReference(CoralConstants.kUppiesL2Height, ControlType.kPosition);
        }
        if(checkUppiesHeight(CoralConstants.kUppiesL2Height)){
            m_wristClosedLoopController.setReference(CoralConstants.kWristL2Angle, ControlType.kPosition);
        }else{
          m_wristClosedLoopController.setReference(CoralConstants.kWristMoveAngle, ControlType.kPosition);
        }
        break;

      //LEVEL 3
      case LEVEL3:
        shootSpeed = CoralConstants.kHouseL3Speed;
        if(checkWristAngle(CoralConstants.kWristMoveAngle)){
          m_uppies1ClosedLoopController.setReference(CoralConstants.kUppiesL3Height, ControlType.kPosition);
        }
        if(checkUppiesHeight(CoralConstants.kUppiesL3Height)){
          m_wristClosedLoopController.setReference(CoralConstants.kWristL3Angle, ControlType.kPosition);
        }else{
          m_wristClosedLoopController.setReference(CoralConstants.kWristMoveAngle, ControlType.kPosition);
        }
        break;

      //LEVEL 4
      case LEVEL4:
        shootSpeed = CoralConstants.kHouseL4Speed;
        if(DriverStation.isAutonomousEnabled()){
          m_wristClosedLoopController.setReference(CoralConstants.kWristL4Angle, ControlType.kPosition);
          if(checkWristAngle(CoralConstants.kWristL4Angle)){
            m_uppies1ClosedLoopController.setReference(CoralConstants.kUppiesL4Height, ControlType.kPosition);
          }
        }else if(DriverStation.isTeleopEnabled()){
          if(checkWristAngle(CoralConstants.kWristMoveAngle)){
            m_uppies1ClosedLoopController.setReference(CoralConstants.kUppiesL4Height, ControlType.kPosition);
          }
          if(checkUppiesHeight(CoralConstants.kUppiesL4Height)){
            m_wristClosedLoopController.setReference(CoralConstants.kWristL4Angle, ControlType.kPosition);
          }else{
            m_wristClosedLoopController.setReference(CoralConstants.kWristMoveAngle, ControlType.kPosition);
          }
        }
        break;
      
      //ALGAE LOW
      case ALGAE1:
        shootSpeed = CoralConstants.kAlgaeShoot;
        if(checkWristAngle(CoralConstants.kWristMoveAngle)) {
          m_uppies1ClosedLoopController.setReference(CoralConstants.kAlgaeRemoveHeight1, ControlType.kPosition);
        }
        if(checkUppiesHeight(CoralConstants.kAlgaeRemoveHeight1)) {
          m_wristClosedLoopController.setReference(CoralConstants.kBallRemoveAngle, ControlType.kPosition);
          m_houseMotor.set(CoralConstants.kRemoveSpeed);
        }else{
          m_wristClosedLoopController.setReference(CoralConstants.kWristMoveAngle, ControlType.kPosition);
        }
        break;

      //ALGAE HIGH
      case ALGAE2:
        shootSpeed = CoralConstants.kAlgaeShoot;
        if(checkWristAngle(CoralConstants.kWristMoveAngle)) {
          m_uppies1ClosedLoopController.setReference(CoralConstants.kAlgaeRemoveHeight2, ControlType.kPosition);
        }
        if(checkUppiesHeight(CoralConstants.kAlgaeRemoveHeight2)) {
          m_wristClosedLoopController.setReference(CoralConstants.kBallRemoveAngle, ControlType.kPosition);
          m_houseMotor.set(CoralConstants.kRemoveSpeed);
        }else{
          m_wristClosedLoopController.setReference(CoralConstants.kWristMoveAngle, ControlType.kPosition);
        }
        break;

      //ALGAE SCORE
      case ALGAESCORE:
        shootSpeed = CoralConstants.kAlgaeShoot;
        m_houseMotor.set(CoralConstants.kRemoveSpeed + 0.05);
        m_uppies1ClosedLoopController.setReference(CoralConstants.kAlgaeScoreHeight, ControlType.kPosition);
        if(checkUppiesHeight(CoralConstants.kAlgaeScoreHeight)) {
          m_wristClosedLoopController.setReference(CoralConstants.kBallRemoveAngle, ControlType.kPosition);
        }
        break;
    }

  }

  public void setState(coralState newState){
    System.out.println("Setting State" + newState);
    desiredState = newState;
  }

  public void setSpeed(double speed){
    autoSpeed = speed;
  }

  boolean checkCoral(){
    boolean coral = !m_houseSwitch.get();
    return coral;
  }

  public void climbMode(){
    climbActivated = !climbActivated;
  }
  
  boolean checkWristAngle(double wristAngle){
    double currentWristAngle = m_wristEncoder.getPosition();
    if(wristAngle <= (currentWristAngle + 15.0) && wristAngle >= (currentWristAngle - 15.0)){
      return true;
    }else{
      return false;
    }
  }

  boolean checkUppiesHeight(double uppiesHeight){
    double currentUppiesHeight = m_uppiesEncoder1.getPosition();
    if(uppiesHeight <= (currentUppiesHeight + 2.0) && uppiesHeight >= (currentUppiesHeight - 2.0)){
      return true;
    }else{
      return false;
    }
  }
}
