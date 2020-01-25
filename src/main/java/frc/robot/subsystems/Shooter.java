/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  final double tkP = 0;
  final double tkI = 0;
  final double tkD = 0;

  final double tkS = 0;
  final double tkV = 0;
  final double tkA = 0;

  final double bkP = 0;
  final double bkI = 0;
  final double bkD = 0;

  final double bkS = 0;
  final double bkV = 0;
  final double bkA = 0;

  final double wheelRadius = Units.inchesToMeters(2);

  WPI_TalonSRX topMotor;
  WPI_TalonSRX bottomMotor;

  PIDController tpid = new PIDController(tkP, tkI, tkD);
  SimpleMotorFeedforward tff = new SimpleMotorFeedforward(tkS, tkV, tkA);
  PIDController bpid = new PIDController(bkP, bkI, bkD);
  SimpleMotorFeedforward bff = new SimpleMotorFeedforward(bkS, bkV, bkA);

  ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  NetworkTableEntry topMotorVoltage = tab.add("Top Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry bottomMotorVoltage = tab.add("Bottom Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry topMotorInverted = tab.add("Top Motor Inverted", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  NetworkTableEntry bottomMotorInverted = tab.add("Bottom Motor Inverted", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  NetworkTableEntry topMotorVelocityGraph = tab.add("Top Motor Velocity Graph (m/s)", 0).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible Time", 10)).getEntry();
  NetworkTableEntry bottomMotorVelocityGraph = tab.add("Bottom Motor Velocity Graph (m/s)", 0).withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible Time", 10)).getEntry();
  NetworkTableEntry topMotorVelocity = tab.add("Top Motor Velocity (m/s)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry bottomMotorVelocity = tab.add("Bottom Motor Velocity (m/s)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

  NetworkTableEntry topSetpoint = tab.add("Top Motor Velocity Setpoint (m/s)", 0).getEntry();
  NetworkTableEntry bottomSetpoint = tab.add("Bottom Motor Velocity Setpoint (m/s)", 0).getEntry();

  NetworkTableEntry manualControl = tab.add("Manual Control", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  NetworkTableEntry topMotorSpeed = tab.add("Set Top Motor Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1, "black increment", 0.1)).getEntry();
  NetworkTableEntry bottomMotorSpeed = tab.add("Set Bottom Motor Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1, "black increment", 0.1)).getEntry();

  

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

    tab.add("Top PID Controller", tpid).withWidget(BuiltInWidgets.kPIDController);
    tab.add("Bottom PID Controller", bpid).withWidget(BuiltInWidgets.kPIDController);

    tpid.setTolerance(10, 20);
    bpid.setTolerance(10, 20);

    // topMotor.setSensorPhase(true);
    // bottomMotor.setSensorPhase(false);
  }


  List<Double> topVelocities = new ArrayList<Double>();
  List<Double> bottomVelocities = new ArrayList<Double>();

  @Override
  public void periodic() {
    topVelocities.add(getTopVelocity());
    bottomVelocities.add(getBottomVelocity());

    double[] topVelocitiesArray = new double[topVelocities.size()];
    double[] bottomVelocitiesArray = new double[bottomVelocities.size()];

    for (int i = 0; i < topVelocities.size(); i ++)
    {
      topVelocitiesArray[i] = topVelocities.get(i);
    }

    for (int i = 0; i < bottomVelocities.size(); i ++)
    {
      bottomVelocitiesArray[i] = topVelocities.get(i);
    }

    // This method will be called once per scheduler run
    topMotor.setInverted(topMotorInverted.getBoolean(true));
    bottomMotor.setInverted(bottomMotorInverted.getBoolean(false));
    topMotorVoltage.setDouble(topMotor.getMotorOutputVoltage());
    bottomMotorVoltage.setDouble(bottomMotor.getMotorOutputVoltage());
    topMotorVelocity.setDouble(getTopVelocity());
    bottomMotorVelocity.setDouble(getBottomVelocity());
    topMotorVelocityGraph.setDoubleArray(topVelocitiesArray);
    bottomMotorVelocityGraph.setDoubleArray(bottomVelocitiesArray);
    System.out.println("Top Motor Velocity " + getTopVelocity());
    boolean isManualControl = manualControl.getBoolean(true);
    

    if (!isManualControl)
    {      
      if (!tpid.atSetpoint()) 
        topMotor.setVoltage(tpid.calculate(getTopVelocity(), topSetpoint.getDouble(0) + tff.calculate(topSetpoint.getDouble(0))));
      
      if (!bpid.atSetpoint())
        bottomMotor.setVoltage(bpid.calculate(getBottomVelocity(), bottomSetpoint.getDouble(0) + bff.calculate(bottomSetpoint.getDouble(0))));
    }
    else
    {
      topMotor.set(topMotorSpeed.getDouble(0));
      bottomMotor.set(bottomMotorSpeed.getDouble(0));
    }
  }

  double getTopVelocity ()
  {
    return topMotor.getSelectedSensorVelocity() * 10 / 4096 * 2 * Math.PI * wheelRadius; 
  }

  double getBottomVelocity ()
  {
    return bottomMotor.getSelectedSensorVelocity() * 10 / 4096 * 2 * Math.PI * wheelRadius; 
  }

}
