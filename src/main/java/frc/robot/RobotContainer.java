// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Limelight;
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
  private final int ElevatorAxis = XboxController.Axis.kRightY.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
  new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton robotCentricBumper =
  new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton resetOdometry = 
  new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton autoAimButton = 
  new JoystickButton(driver, XboxController.Button.kA.value);
  //private final JoystickButton xSwerve = 
  //new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private boolean robotCentric = false;

  private final JoystickButton AimBot = 
  new JoystickButton(operator, XboxController.Button.kB.value);

  /* Subsystems */
  private DigitalInput lightbreakSensor;
  private final Limelight speakerLimelight = new Limelight("limelight");
  private final Limelight noteLimelight = new Limelight("NoteVision");


  /* Robot Variables */
  private final SendableChooser<Command> autoChooser;

  public enum ShooterState{
    Off,
    Intake,
    ReadyToShoot,
    Shoot
  }
  public ShooterState state;
  public double shooterVoltage;

  public RobotContainer() {
    
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    robotCentricBumper.onTrue(new InstantCommand(() -> {
      robotCentric = !robotCentric;
      SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
    }));

    resetOdometry.onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));
    

    /* Operator Buttons */
 }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void teleopInit(){
    swerve.xPatternFalse();
    swerve.resetToAbsolute();

    swerve.setDefaultCommand(
      new TeleopSwerve(
        swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> (autoAimButton.getAsBoolean() ? -speakerLimelight.getRobotRotationtoSpeaker()*Preferences.getDouble("AutoAimStrength", 1.0)/100.0 : -driver.getRawAxis(rotationAxis)),
        () -> robotCentric));
  }

  public void teleopExit() {
    swerve.removeDefaultCommand();
  }

  public void autoInit(){
    swerve.resetToAbsolute();
  }

  public void testInit(){
    swerve.xPatternFalse();
    swerve.resetToAbsolute();
    CommandScheduler.getInstance().schedule(swerve.getTestCommand());
  }
  
\


}
