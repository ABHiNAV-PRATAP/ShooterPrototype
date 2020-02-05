/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  WPI_VictorSPX intakeVictor;

  public Intake() {
    intakeVictor = new WPI_VictorSPX(5);
    intakeVictor.setInverted(true);
    // invrert if needed


  }

  // -1 : 1
  public void setValue(double value) {
    intakeVictor.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
