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
import frc.robot.subsystems.ShooterWheels;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.DefaultCommands.IntakeDefault;
import frc.robot.commands.DefaultCommands.ShooterDefault;
import frc.robot.commands.DefaultCommands.FeederDefault;
import frc.robot.commands.DefaultCommands.PivotDefault;
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
  private final JoystickButton toggleShooter =
  new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton useFeeder =
  new JoystickButton(operator, XboxController.Button.kY.value);

  private final JoystickButton angleButton1 =
  new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton angleButton2 =
  new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton angleButton3 =
  new JoystickButton(operator, XboxController.Button.kB.value);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake(); 
  private final ShooterWheels shooter = new ShooterWheels();
  private final Feeder feeder = new Feeder();
  private final Pivot pivot = new Pivot();

  /* Robot Variables */
  public boolean intakeActive = false;
  public boolean shooterActive = false;

  public double targetAngle = 0;
  
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
        () -> Constants.Swerve.maxSpeed * Math.sqrt((driver.getRawAxis(translationAxis) * driver.getRawAxis(translationAxis)) + (driver.getRawAxis(strafeAxis) * driver.getRawAxis(strafeAxis)))
      )
    );

    shooter.setDefaultCommand(
      new ShooterDefault(
        shooter,
        () -> shooterActive,
        () -> Constants.Electical.shooterHardcodedVoltage
      )
    );

    feeder.setDefaultCommand(
      new FeederDefault(
        feeder,
        () -> useFeeder.getAsBoolean(),
        () -> Constants.Electical.feederHarcodedVoltage
      )
    );

    pivot.setDefaultCommand(
      new PivotDefault(
        pivot,
        () -> angleButton1.getAsBoolean(),
        () -> angleButton2.getAsBoolean(),
        () -> angleButton3.getAsBoolean(),
        () -> targetAngle
      )
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
    toggleShooter.onTrue(new InstantCommand(() -> toggleShooter()));

    angleButton1.onTrue(new InstantCommand(() -> setAngle(1)));
    angleButton2.onTrue(new InstantCommand(() -> setAngle(2)));
    angleButton2.onTrue(new InstantCommand(() -> setAngle(3)));
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

  public void toggleShooter(){
    shooterActive = !shooterActive;
  }

  public void setAngle(int angleNum){

    // SIDENOTE: This is like. the worst possible way of doing this.
    //           Either rewrite this or get rid of it entirely.

    switch (angleNum) {
      case 1:
        targetAngle = 0;
        break;
      case 2:
        targetAngle = 0.1;
        break;
      case 3:
        targetAngle = -0.2;
        break;
      default:
        break;
    }
  }
}
