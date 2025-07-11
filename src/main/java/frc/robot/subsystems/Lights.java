// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.subsystems.CoralSubsystem;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  AddressableLED m_LED = new AddressableLED(OIConstants.kLEDPort);
  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(DriveConstants.kLEDLength);
  CoralSubsystem coralSubsystem;

  boolean leftAutoAlign;
  boolean rightAutoAlign;
  int disableLoopCount;
  boolean climbModeActivated = false;

  public Lights() {
    for (int i = 0; i < DriveConstants.kLEDLength - 1; i = i + 2){
      m_buffer.setRGB(i, 0, 133, 202);
      m_buffer.setRGB((i + 1), 252, 186, 3);
    }
    m_LED.setLength(DriveConstants.kLEDLength);
    m_LED.setData(m_buffer);
    m_LED.start();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(coralSubsystem.haveCoral){
      for(int i = 0; i < DriveConstants.kLEDLength; i++){
        m_buffer.setRGB(i, 201, 5, 255);
      }
    }else{
      for (int i = 0; i < DriveConstants.kLEDLength - 1; i = i + 2){
        m_buffer.setRGB(i, 0, 133, 202);
        m_buffer.setRGB((i + 1), 252, 186, 3);
      }
    }
    m_LED.setData(m_buffer);
    m_LED.start();
  }

  public void autoAlignLeft(boolean left){
    leftAutoAlign = left;
  }

  public void autoAlignRight(boolean right){
    rightAutoAlign = right;
  }

  public void climbMode(){
    climbModeActivated = !climbModeActivated;
  }
}
