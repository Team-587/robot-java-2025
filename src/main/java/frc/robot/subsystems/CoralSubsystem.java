// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkFlex;
//import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

import com.fasterxml.jackson.core.util.Separators;
import com.revrobotics.AnalogInput;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  enum coralState {
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

  }

   coralState desiredState;
   boolean haveCoral;
   boolean climbActivated;
   double autoSpeed;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setState(coralState newState){

  }

  public void setSpeed(double speed){

  }

  public void climbMode(){

  }
  
  boolean checkWristAngle(double wristAngle){
    return false;
  }

  boolean checkUppiesHeight(double uppiesHeight){
    return false;
  }
}
