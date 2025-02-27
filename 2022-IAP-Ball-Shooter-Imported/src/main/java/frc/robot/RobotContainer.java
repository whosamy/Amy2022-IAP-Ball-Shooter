// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Shoot lowShoot = new Shoot(Constants.lowSpeed);
  private final Shoot midShoot = new Shoot(Constants.midSpeed);
  private final Shoot highShoot = new Shoot(Constants.highSpeed);
  private static Joystick joystick;
  private static JoystickButton lowSpeedButton;
  private static JoystickButton midSpeedButton;
  private static JoystickButton highSpeedButton;
  private final static BallShooter ballShooter = new BallShooter();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    joystick = new Joystick(Constants.joystick);
    lowSpeedButton = new JoystickButton(joystick, Constants.lowSpeedButton);
    midSpeedButton = new JoystickButton(joystick, Constants.midSpeedButton);
    highSpeedButton = new JoystickButton(joystick, Constants.highSpeedButton);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    lowSpeedButton.whenPressed(lowShoot);
    midSpeedButton.whenPressed(midShoot);
    highSpeedButton.whenPressed(highShoot);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Joystick getJoystick(){
    return joystick;
  }
  public static BallShooter getBallShooter(){
    return ballShooter;
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
