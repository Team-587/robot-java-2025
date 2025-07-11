// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.AnalogInput;
import edu.wpi.first.wpilibj.Servo;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final SparkMax m_climberMotor;
  private final Servo m_hopperServo;
  private final Servo m_climberServo;

  XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  boolean canClimb;
  boolean xIsPressed;

  public Climber() {
    canClimb = false;
    xIsPressed = false;

    m_climberMotor = new SparkMax(DriveConstants.kClimberCanId, MotorType.kBrushless);
    m_hopperServo = new Servo(DriveConstants.kHopperServoId);
    m_climberServo = new Servo(DriveConstants.kClimberServoId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!canClimb){
      m_climberServo.set(0.65);
      m_climberMotor.stopMotor();
    }else if (canClimb){
      if(getLeftTriggerPressed()){
        climberDown();
      }else if(getRightTriggerPressed()){
        climberIn();
      }else if(!getLeftTriggerPressed() && !getRightTriggerPressed()){
        m_climberServo.set(0.0);
      }
    }
  }

  public void allowClimn(){
    canClimb = !canClimb;
  }

  public void dropHopper(){
    if(canClimb){
      m_hopperServo.set(0);
    }
  }

  public void climberIn(){
    if(canClimb){
      m_climberServo.set(0.65);
      m_climberMotor.set(m_coDriverController.getRightTriggerAxis());
    }
  }

  public void climberOut(){
    if(canClimb){
      m_climberMotor.set(m_coDriverController.getLeftTriggerAxis() * -0.65);
    }
  }

  public boolean climbMode(){
    return canClimb;
  }

  public boolean getLeftTriggerPressed(){
    if(m_coDriverController.getLeftTriggerAxis() >= 0.2){
      return true;
    }else{
      return false;
    }
  }

  public boolean getRightTriggerPressed(){
    if(m_coDriverController.getRightTriggerAxis() >= 0.2){
      return true;
    }else{
      return false;
    }
  }

  public void climberDown(){
    m_climberServo.set(0);
    sleep(500);
    climberOut();
  }

  public void sleep(int time){
    try {
      Thread.sleep(time);
    } catch (Exception e) {} 
  }
}
