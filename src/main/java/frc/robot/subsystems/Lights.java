// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  AddressableLED m_LED = new AddressableLED(OIConstants.kLEDPort);
  AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(DriveConstants.kLEDLength);

  public Lights() {
    
  }
  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
