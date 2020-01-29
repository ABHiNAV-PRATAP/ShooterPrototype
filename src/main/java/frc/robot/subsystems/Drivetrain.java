/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  
  WPI_TalonSRX fLMaster, fRMaster;
  WPI_VictorSPX bLSlave, bRSlave;

  AHRS gyro;

  public Drivetrain() {
    fLMaster = new WPI_TalonSRX(1);
    bLSlave = new WPI_VictorSPX(2);
    fRMaster = new WPI_TalonSRX(3);
    bRSlave = new WPI_VictorSPX(4);

    gyro = new AHRS(Port.kMXP);

    bLSlave.follow(fLMaster);
    bRSlave.follow(fRMaster);

    fRMaster.setInverted(true);
    bRSlave.setInverted(true);

  fLMaster.setSelectedSensorPosition(0);
  fRMaster.setSelectedSensorPosition(0);

  gyro.reset();

    fLMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    fRMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void setMotorSpeed(double left, double right)
  {
    fRMaster.set(right);
    fLMaster.set(left);
  }

  @Override
  public void periodic() {
    System.out.println(gyro.getAngle());
    // System.out.println("Left: " + fLMaster.getSelectedSensorPosition());
    // System.out.println("Right: " + fRMaster.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }
}
