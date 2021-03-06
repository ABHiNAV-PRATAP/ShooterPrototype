/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.DavidDrive;
import frc.robot.commands.intake.IntakeCell;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Shooter shooter = new Shooter();

  private final Intake intake = new Intake();
  
  private final JoystickButton shoot;

  private final JoystickButton intakeCell;

  private final Drivetrain drivetrain = new Drivetrain();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private Joystick driveJoystick = new Joystick(0);

  private final XboxController controller = new XboxController(1);

  public static final PowerDistributionPanel PDP = new PowerDistributionPanel();



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    shoot = new JoystickButton(driveJoystick, 1);

    intakeCell = new JoystickButton(driveJoystick, 2);
    // Configure the button bindings

    // shooter.setDefaultCommand(
    //   new Shoot(
    //     shooter,
    //     () -> shooter.topSetpointShuffleboard.getDouble(0),
    //     () -> shooter.bottomSetpointShuffleboard.getDouble(0)
    //   )
    // );
    
    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        () -> controller.getY(Hand.kRight), 
        () -> controller.getX(Hand.kRight),
        () -> driveJoystick.getThrottle()
      )
    );
    
    
    // drivetrain.setDefaultCommand(
    //   new ArcadeDrive(
    //     drivetrain,
    //     () -> controller.getY(Hand.kLeft), 
    //     () -> controller.getX(Hand.kLeft),
    //     () -> controller.getTriggerAxis(Hand.kLeft)
    //   )
    // );
    
    
    // drivetrain.setDefaultCommand(
    //   new DavidDrive(
    //     drivetrain,
    //     () -> controller.getTriggerAxis(Hand.kRight),
    //     () -> controller.getTriggerAxis(Hand.kLeft),
    //     () -> controller.getX(Hand.kLeft)
    //   )
    // );
    

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    shoot.whileHeld(new Shoot(shooter, () -> shooter.topSetpointShuffleboard.getDouble(0), () -> shooter.bottomSetpointShuffleboard.getDouble(0)).andThen(() -> shooter.setBottomMotorVoltage(0)).andThen(() -> shooter.setTopMotorVoltage(0)));
    intakeCell.whileHeld(new IntakeCell(intake));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
