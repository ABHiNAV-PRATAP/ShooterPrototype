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

public class ArcadeDrive extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   */

   Drivetrain drivetrain;
   DoubleSupplier throttleSupplier;
   DoubleSupplier turnSupplier;
   DoubleSupplier accelerationSupplier;

  public ArcadeDrive(Drivetrain drive, DoubleSupplier throttle, DoubleSupplier turn, DoubleSupplier accel) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = drive;
    throttleSupplier = throttle;
    turnSupplier = turn;
    accelerationSupplier = accel;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplierRaw = accelerationSupplier.getAsDouble();
    double throttle = throttleSupplier.getAsDouble();
    double turn = turnSupplier.getAsDouble();
    
    double speedMultiplier = MathUtil.normalize(1, -1, 0.2, 1, speedMultiplierRaw);
    // System.out.println("Speed Multiplier " + speedMultiplier);
    // System.out.println("X " + turn);
    // System.out.println("Y " + throttle);

    drivetrain.setMotorSpeed((throttle + turn) * speedMultiplier, (throttle - turn) * speedMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
