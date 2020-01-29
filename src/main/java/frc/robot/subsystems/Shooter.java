/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  double tkP = 0.127;
  final double tkI = 0;
  double tkD = 0;

  final double tkS = 0.6;
  final double tkV = 0.13;
  final double tkA = 0.05;

  double bkP = 0.127;
  final double bkI = 0;
  double bkD = 0;

  final double bkS = 0.0886;
  final double bkV = 0.14;
  final double bkA = 0.05;

  final double wheelRadius = Units.inchesToMeters(2);

  WPI_TalonSRX topMotor;
  WPI_TalonSRX bottomMotor;
  // Servo servo = new Servo(0);

  public PIDController tpid = new PIDController(tkP, tkI, tkD);
  public SimpleMotorFeedforward tff = new SimpleMotorFeedforward(tkS, tkV, tkA);
  public PIDController bpid = new PIDController(bkP, bkI, bkD);
  public SimpleMotorFeedforward bff = new SimpleMotorFeedforward(bkS, bkV, bkA);

  public ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  NetworkTableEntry topMotorVoltage = tab.add("Top Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry bottomMotorVoltage = tab.add("Bottom Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry topMotorVelocity = tab.add("TopVelocity (rps)", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry bottomMotorVelocity = tab.add("BottomVelocity (rps)", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();

  NetworkTableEntry bottomP = tab.add("bottom P", bkP).withWidget(BuiltInWidgets.kTextView).getEntry();

  public NetworkTableEntry topSetpointShuffleboard = tab.add("Top Setpoint", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
  public NetworkTableEntry bottomSetpointShuffleboard = tab.add("Bottom Setpoint", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
  // public NetworkTableEntry servoAngle = tab.add("Servo Angle", 0).withWidget(BuiltInWidgets.kTextView).getEntry();


  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    topMotor = new WPI_TalonSRX(10);
    bottomMotor = new WPI_TalonSRX(11);

    topMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    topMotor.setSelectedSensorPosition(0);
    bottomMotor.setSelectedSensorPosition(0);
    topMotor.setInverted(true);
    bottomMotor.setInverted(false);

    

    tpid.setTolerance(1, 1);
    bpid.setTolerance(1, 1);
  }

  public void setTopMotorVoltage(double value) {
    topMotor.setVoltage(value);
  }

  public void setServoAngle(double degrees)
  {
    // servo.setAngle(degrees);
  }

  public void setBottomMotorVoltage(double value) {
    bottomMotor.setVoltage(value);
  }

  @Override
  public void periodic() {
    topMotorVoltage.setDouble(topMotor.getMotorOutputVoltage());
    bottomMotorVoltage.setDouble(bottomMotor.getMotorOutputVoltage());
    topMotorVelocity.setDouble(getTopVelocity());
    bottomMotorVelocity.setDouble(getBottomVelocity());
    bpid.setP(bottomP.getDouble(bkP));
  }

  public double getTopVelocity ()
  {
    return topMotor.getSelectedSensorVelocity() * 10 / 4096; 
  }

  public double getBottomVelocity ()
  {
    return bottomMotor.getSelectedSensorVelocity() * 10 / 4096; 
  }

}
