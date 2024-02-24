// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import frc.robot.commands.DefaultCommands.IntakeDefault;
import frc.robot.commands.DefaultCommands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
  new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton robotCentricBumper =
  new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton resetOdometry = 
  new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton xSwerve = 
  new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private boolean robotCentric = false;

  /* Operator Buttons */
  private final JoystickButton toggleIntake =
  new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake(); 

  /* Robot Variables */
  public boolean intakeActive = false;
  
  public RobotContainer() {
    swerve.setDefaultCommand(
      new TeleopSwerve(
        swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> robotCentric));

    intake.setDefaultCommand(
      new IntakeDefault(
        intake, 
        () -> intakeActive,
        () -> Constants.Swerve.maxSpeed * Math.sqrt((driver.getRawAxis(translationAxis) * driver.getRawAxis(translationAxis)) + (driver.getRawAxis(strafeAxis) * driver.getRawAxis(strafeAxis))))
    );

    configureBindings();    
  }

  private void configureBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    robotCentricBumper.onTrue(new InstantCommand(() -> {
      robotCentric = !robotCentric;
      SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
    }));

    resetOdometry.onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));
    xSwerve.onTrue(new InstantCommand(() -> swerve.xPattern()));

    /* Operator Buttons */
    toggleIntake.onTrue(new InstantCommand(() -> toggleIntake()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit(){
    swerve.xPatternFalse();
    swerve.resetToAbsolute();
  }

  public void autoInit(){
    swerve.resetToAbsolute();
  }

  public void toggleIntake(){
    intakeActive = !intakeActive;
  }
}
