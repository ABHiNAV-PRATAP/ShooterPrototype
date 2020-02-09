/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// neos are spinning backwards so fix tht, spacer is needed 
package frc.robot.subsystems;
import java.util.List;
import java.util.Map;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Shooter extends SubsystemBase {
  // List<double[]> targetPoseSample;
  double[] targetPose;
  double tkP = 0.0075; //0.957;
  final double tkI = 0;
  double tkD = 0;
  final double tkS = 0.0643; //0.0643
  final double tkV = 0.128; //0.128
  final double tkA = 0.0205; //0.0205
  double bkP = 0; // -0.0075; // .01
  final double bkI = 0;
  double bkD = 0;
  final double bkS = 0.0496; // 0.0475; //0.136
  final double bkV = 0.129;// 0.134; //0.128
  final double bkA = 0.0208; // 0.0264; //.0272
  // new characterization
  // forward ks = 0.0523, kv = 0.128, ka = 0.0202, kp = 0.943
  // backward ks = 0.0496, kv = 0.129, ka = 0.0208, kp = 0.972
  final double wheelRadius = Units.inchesToMeters(2);
  CANSparkMax topMotor;
  CANSparkMax bottomMotor;
  Servo servo = new Servo(0);
  public PIDController tpid = new PIDController(tkP, tkI, tkD);
  public SimpleMotorFeedforward tff = new SimpleMotorFeedforward(tkS, tkV, tkA);
  public PIDController bpid = new PIDController(bkP, bkI, bkD);
  public SimpleMotorFeedforward bff = new SimpleMotorFeedforward(bkS, bkV, bkA);
  public ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  NetworkTableEntry topMotorVoltage = tab.add("Top Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry bottomMotorVoltage = tab.add("Bottom Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry topMotorVelocity = tab.add("t vel", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry bottomMotorVelocity = tab.add("b vel", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry topMotorError = tab.add("t err", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry bottomMotorError = tab.add("b err", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry bottomP = tab.add("bottom P", bkP).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry topP = tab.add("top P", tkP).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry bottomD = tab.add("bottom D", bkD).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry topS = tab.add("top S", tkS).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry bottomS = tab.add("bottom S", bkS).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry topV = tab.add("top V", tkV).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry bottomV = tab.add("bottom V ", bkV).withWidget(BuiltInWidgets.kTextView).getEntry();
  public NetworkTableEntry topSetpointShuffleboard = tab.add("Top Setpoint", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
  public NetworkTableEntry bottomSetpointShuffleboard = tab.add("Bottom Setpoint", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
  // Gets the default instance of NetworkTables
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  // Gets the MyCamName table under the chamelon-vision table
  // MyCamName will vary depending on the name of your camera
  NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("infuzed-ps3");
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    topMotor = new CANSparkMax(11, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(10, MotorType.kBrushless);
    // topMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // topMotor.setSelectedSensorPosition(0);
    // bottomMotor.setSelectedSensorPosition(0);
    //topMotor.setInverted(true);
    //bottomMotor.setInverted(false);
    topMotor.setInverted(false);
    bottomMotor.setInverted(true);
    // tpid.setTolerance(1, 1);
    // bpid.setTolerance(1, 1);
    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPosition(0);
  }
  public void setTopMotorVoltage(double value) {
    topMotor.setVoltage(value);
  }
  public void setServoAngle(double degrees) {
    servo.setAngle(degrees);
  }
  public void setBottomMotorVoltage(double value) {
    bottomMotor.set(value/12);
  }
  public Double[] getTargetPose() {
    return cameraTable.getEntry("targetPose").getDoubleArray(new Double[] {0.0, 0.0, 0.0});
  }
  public Double getDistanceFromPose() {
    return getTargetPose()[0]*1.613191185 - 1.267661978;// old regression: 1.332335981 - 0.0744373614;
  }
  @Override
  public void periodic() {
    topMotorVoltage.setDouble(topMotor.getBusVoltage());
    bottomMotorVoltage.setDouble(bottomMotor.getBusVoltage());
    topMotorVelocity.setDouble(getTopVelocity());
    bottomMotorVelocity.setDouble(getBottomVelocity());
    tpid.setP(topP.getDouble(tkP));
    bpid.setP(bottomP.getDouble(bkP));
    bpid.setD(bottomP.getDouble(bkD));
    bff = new SimpleMotorFeedforward(bottomP.getDouble(0.126), bkV);
    System.out.println(getDistanceFromPose());
    // Double[] pose = getTargetPose();
    // System.out.println("x" + pose[0]);
    // System.out.println("y" + pose[1]);
    // System.out.println("angle" + pose[2]);
// System.out.println(bff.ks);
    // System.out.println("Top encoder ticks:" + topMotor.getEncoder().getPosition());
    // System.out.println("Bottom encoder ticks: " + bottomMotor.getEncoder().getPosition());
  }
  public double getTopVelocity() {
    return topMotor.getEncoder().getVelocity() / 60;
  }
  public double getBottomVelocity() {
    return bottomMotor.getEncoder().getVelocity() / 60;
  }
}