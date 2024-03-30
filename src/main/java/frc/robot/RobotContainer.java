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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterWheels;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.DefaultCommands.IntakeDefault;
import frc.robot.commands.DefaultCommands.ShooterDefault;
import frc.robot.commands.DefaultCommands.ElevatorDefault;
import frc.robot.commands.DefaultCommands.FeederDefault;
import frc.robot.commands.DefaultCommands.PivotDefault;
import frc.robot.commands.Rotate;
import frc.robot.commands.SetPivotCommand;
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
  new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton xSwerve = 
  new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private boolean robotCentric = false;

  /* Operator Buttons */
  private final JoystickButton startIntake =
  new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton reverseIntakeButton =
  new JoystickButton(operator, XboxController.Button.kBack.value);
  private final JoystickButton shootNote =
  new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton stopButton =
  new JoystickButton(operator, XboxController.Button.kY.value);

  private final JoystickButton pivotDefaultButton =
  new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton pivotPos1Button =
  new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton pivotPos2Button =
  new JoystickButton(operator, XboxController.Button.kA.value);

  /* Subsystems */
  private DigitalInput lightbreakSensor;
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake(); 
  private final ShooterWheels shooter = new ShooterWheels();
  private final Feeder feeder = new Feeder();
  private final Pivot pivot = new Pivot();
  private final Elevator elevator = new Elevator();
  private final Limelight limelight = new Limelight();


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

  public boolean reverseIntake;

  public boolean reverseFeeder;

  public RobotContainer() {
    // Initialize Autonomous Commands
    NamedCommands.registerCommand("AutoIntake", new InstantCommand(() -> changeShooterState(ShooterState.Intake))
     .andThen(new SetPivotCommand(pivot, Constants.Pivot.intakeAngle, () -> operator.getRawAxis(translationAxis))));

    NamedCommands.registerCommand("AutoReadyToShoot", new InstantCommand(() -> changeShooterState(ShooterState.ReadyToShoot))
    .andThen(new SetPivotCommand(pivot, Constants.Pivot.speakerShotAngle, () -> operator.getRawAxis(translationAxis))));

    NamedCommands.registerCommand("AutoShoot", new InstantCommand(() -> changeShooterState(ShooterState.Shoot)));
    NamedCommands.registerCommand("AutoShooterStop", new InstantCommand(() -> changeShooterState(ShooterState.Off)));

    //Enable when is needed Aim long distance shoot

    //NamedCommands.registerCommand("AimLongDistance", new InstantCommand(()-> shooterVoltage = Constants.Shooter.longshotVoltage).andThen(
    //new SetPivotCommand(pivot,Constants.Pivot.speakerShotAngle + 10,()-> 0)));

    shooterVoltage = Constants.Shooter.speakershotVoltage;

    // Channel and set up for Lightbreak Sensor
    lightbreakSensor = new DigitalInput(0);

    Trigger haveNote = new Trigger(lightbreakSensor::get).negate();

    haveNote.onTrue(new InstantCommand(() -> {
      changeShooterState(ShooterState.Off);
      SmartDashboard.putBoolean("lightbreak", lightbreakSensor.get());
    }));

    haveNote.onFalse(new WaitCommand(0.7).andThen(new InstantCommand(() -> {
      changeShooterState(ShooterState.Off);
      SmartDashboard.putBoolean("lightbreak", lightbreakSensor.get());
    })));

    swerve.setDefaultCommand(
      new TeleopSwerve(
        swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> robotCentric));

    intake.setDefaultCommand(

      //New, dumb code that works
      new IntakeDefault(
        intake, 
        () -> (state == ShooterState.Intake),
        () -> Constants.Intake.intakeSpeed,
        () -> reverseIntakeButton.getAsBoolean()
      )
    );

    shooter.setDefaultCommand(
      new ShooterDefault(
        shooter,
        () -> (state == ShooterState.ReadyToShoot || state == ShooterState.Shoot),
        () -> shooterVoltage
      )
    );

    feeder.setDefaultCommand(

      new FeederDefault(
        feeder,
        () -> (state == ShooterState.Intake || state == ShooterState.Shoot),
        () -> Constants.Feeder.feederPower,
        () -> reverseIntakeButton.getAsBoolean()
      )
    );

    // Allows for joystick control
    
    pivot.setDefaultCommand(
      new PivotDefault(
        pivot,
        () -> -operator.getRawAxis(translationAxis)
      )
    );

    if (!Preferences.containsKey("ElevatorStrength")) {
      Preferences.setDouble("ElevatorStrength", 0.1);
    }
    
    elevator.setDefaultCommand(
      new ElevatorDefault(
        elevator,
        // () -> 0.0
        () -> -operator.getRawAxis(ElevatorAxis)*Preferences.getDouble("ElevatorStrength", 0.4) // Reversed controller axis to be correct
      )
    );

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
    xSwerve.onTrue(new InstantCommand(() -> swerve.xPattern()));

    /* Operator Buttons */
    startIntake.onTrue(new InstantCommand(() -> changeShooterState(ShooterState.Intake)).andThen(
      new SetPivotCommand(pivot, Constants.Pivot.intakeAngle, () -> operator.getRawAxis(translationAxis))));

    shootNote.onTrue(new InstantCommand(() -> changeShooterState(ShooterState.ReadyToShoot)).andThen(
      new WaitCommand(0.7).andThen(
      new InstantCommand(() -> changeShooterState(ShooterState.Shoot)))));
      
    stopButton.onTrue(new InstantCommand(() -> changeShooterState(ShooterState.Off)));

    pivotDefaultButton.onTrue(new SetPivotCommand(pivot, Constants.Pivot.intakeAngle, () -> operator.getRawAxis(translationAxis)));
    pivotPos1Button.onTrue( new InstantCommand(()-> shooterVoltage = Constants.Shooter.speakershotVoltage ).andThen(
      new SetPivotCommand(pivot, Constants.Pivot.speakerShotAngle, () -> operator.getRawAxis(translationAxis))));
    pivotPos2Button.onTrue( new InstantCommand(() -> shooterVoltage = Constants.Shooter.longshotVoltage ).andThen(
      new SetPivotCommand(pivot, Constants.Pivot.speakerShotAngle + 10, () -> operator.getRawAxis(translationAxis))));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void teleopInit(){
    swerve.xPatternFalse();
    swerve.resetToAbsolute();
  }

  public void autoInit(){
    swerve.resetToAbsolute();
  }

  public void testInit(){
    swerve.xPatternFalse();
    swerve.resetToAbsolute();
    CommandScheduler.getInstance().schedule(swerve.getTestCommand());
  }
  
  public void changeShooterState(ShooterState changeto) {
    state = changeto;
    SmartDashboard.putString("ShooterState", state.name());
  }

  public void autoShoot() {
    // [0]: angle, [1]: rotation
    double[] result = limelight.getPivotAngletoSpeaker();
    new Rotate(swerve, result[1], robotCentricBumper);
    new SetPivotCommand(pivot, result[0]);
  }
}
