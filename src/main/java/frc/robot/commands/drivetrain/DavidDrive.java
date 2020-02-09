/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.MathUtil;

public class DavidDrive extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   */

   Drivetrain drivetrain;
   DoubleSupplier throttleSupplier;
   DoubleSupplier turnSupplier;
   DoubleSupplier reverseSupplier;
   private static double throttleAccumulator = 0.0;
   private static boolean brake = false;

  public DavidDrive(Drivetrain drive, DoubleSupplier throttle, DoubleSupplier reverse, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    throttleSupplier = throttle;
    turnSupplier = turn;
    reverseSupplier = reverse;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    throttleAccumulator = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = -throttleSupplier.getAsDouble() + reverseSupplier.getAsDouble();
    if(throttleAccumulator<0 && throttle>0){
      brake = true;
    }
    throttleAccumulator += (throttle*.2);
    throttleAccumulator*=0.98;
    if(throttleAccumulator>1.0){
      throttleAccumulator = 1.0;
    }
    if(throttleAccumulator<-1.0){
      throttleAccumulator = -1.0;
    }
    if(brake == true){
      if(throttleAccumulator>0.0){
        throttleAccumulator=0.0;
      }
      if(Math.abs(throttle)<0.01) {
        brake = false;
      }
    }
    double turn = turnSupplier.getAsDouble();
    System.out.println(throttleAccumulator);
    // System.out.println("Speed Multiplier " + speedMultiplier);
    // System.out.println("X " + turn);
    // System.out.println("Y " + throttle);

    drivetrain.setMotorSpeed((throttleAccumulator + turn) * Math.abs(throttleAccumulator), (throttleAccumulator - turn) * Math.abs(throttleAccumulator));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    throttleAccumulator = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
