// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SetElevator;
import frc.robot.commands.DefaultCommands.ElevatorDefault;
import frc.robot.commands.DefaultCommands.TeleopSwerve;
import frc.robot.commands.DefaultCommands.WristDefault;
import frc.robot.Constants.Level;
import frc.robot.Constants.Location;

public class RobotContainer {

  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  /** Driver - Back (Minus) */
  private final JoystickButton zeroGyro =
  new JoystickButton(driver, XboxController.Button.kBack.value);
  /** Driver - Start (Plus) */
  private final JoystickButton robotCentricBumper =
  new JoystickButton(driver, XboxController.Button.kStart.value);
  /** Driver - Y (X on our controller) */
  private final JoystickButton resetOdometry = 
  new JoystickButton(driver, XboxController.Button.kY.value);
  //private final JoystickButton autoAimButton = 
  //new JoystickButton(driver, XboxController.Button.kA.value);
  private boolean robotCentric = false;

  private final JoystickButton swerve_quasiF = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton swerve_quasiR = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton swerve_dynF = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton swerve_dynR = new JoystickButton(driver, XboxController.Button.kY.value);

  private final JoystickButton elevator_quasiF = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton elevator_quasiR = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton elevator_dynF = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton elevator_dynR = new JoystickButton(operator, XboxController.Button.kY.value);

  /* Operator Buttons */
  /** Operator - Left Stick X */
  private final int reefAxis = XboxController.Axis.kLeftX.value;
  /** Operator - Left Stick Y */
  private final int wristAxis = XboxController.Axis.kLeftY.value;
  /** Operator - Right Stick Y */
  private final int elevatorAxis = XboxController.Axis.kRightY.value;
  // private final int leftReefAxis = XboxController.Axis.kLeftTrigger.value;
  // private final int rightReefAxis = XboxController.Axis.kRightTrigger.value;
  /** Operator - Back (Minus) */
  private final JoystickButton coralButton = 
  new JoystickButton(operator, XboxController.Button.kBack.value);
  /** Operator - Start (Plus) */
  private final JoystickButton algaeButton = 
  new JoystickButton(operator, XboxController.Button.kStart.value);
  /** Operator - Left Bumper */
  private final JoystickButton interruptButton =
  new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  /** Operator - A Button (B) */
  private final JoystickButton elevFloorButton =
  new JoystickButton(operator, XboxController.Button.kA.value);
  /** Operator - X Button (Y) */
  private final JoystickButton elevL2Button =
  new JoystickButton(operator, XboxController.Button.kX.value);
  /** Operator - B Button (A) */
  private final JoystickButton elevL3Button =
  new JoystickButton(operator, XboxController.Button.kB.value);
  /** Operator - Y Button (X) */
  private final JoystickButton elevL4Button =
  new JoystickButton(operator, XboxController.Button.kY.value);
  /** Operator - Left Trigger */
  private final int intakeTrigger = XboxController.Axis.kLeftTrigger.value;
  private final Trigger clawIntake = new Trigger(() -> operator.getRawAxis(intakeTrigger) > 0.1);
  /** Operator - Right Trigger */
  private final int releaseTrigger = XboxController.Axis.kRightTrigger.value;
  private final Trigger clawReverse = new Trigger(() -> operator.getRawAxis(releaseTrigger) > 0.1);

  /* Subsystems */
  private final DigitalInput beambreak = new DigitalInput(Constants.Setup.beambreakID);


  // If we add a physical switch for algae detection, uncomment these, and comment the other haveGamePiece trigger
  // private final DigitalInput physicalSwitch = new DigitalInput(Constants.Setup.physicalSwitchID);
  // private final Trigger haveGamePiece = new Trigger(() -> beambreak.get()).or(() -> physicalSwitch.get());
  private final Trigger haveGamePiece = new Trigger(() -> beambreak.get());

  private final Swerve swerve = new Swerve();
  // private final Limelight s_Limelight = new Limelight("limelight");
  private final Wrist s_Wrist = new Wrist();
  private final Claw s_Claw = new Claw();
  private final Elevator s_Elevator = new Elevator(s_Wrist);

  /* Robot Variables */
  //private final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    
    configureBindings();

    /* Preferences initialization */
    if (!Preferences.containsKey("ElevatorVoltage")) {
      Preferences.initDouble("ElevatorVoltage", 1);
    }
    if (!Preferences.containsKey("ElevatorGravity")) {
      Preferences.initDouble("ElevatorGravity", 0.3);
    }
    if (!Preferences.containsKey("ClawSpeed")) {
      Preferences.initDouble("ClawSpeed", 0.2);
    }
    if (!Preferences.containsKey("WristSpeed")) {
      Preferences.initDouble("WristSpeed", 0.2);
    }

    /* 
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    */
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
    /* Once claw is installed: */
    clawIntake.onTrue(new InstantCommand(() -> s_Claw.setVoltage(Preferences.getDouble("ClawSpeed", 0.2))));
    clawReverse.onTrue(new InstantCommand(() -> s_Claw.setVoltage(-Preferences.getDouble("ClawSpeed", 0.2))));
    
    elevFloorButton.onTrue(new SetElevator(s_Elevator, Constants.Level.Retracted, interruptButton));
    elevL2Button.onTrue(new SetElevator(s_Elevator, Constants.Level.L2, interruptButton));
    elevL3Button.onTrue(new SetElevator(s_Elevator, Constants.Level.L3, interruptButton));
    elevL4Button.onTrue(new SetElevator(s_Elevator, Constants.Level.L4, interruptButton));
    
    /* Once beambreak is installed */
    haveGamePiece.onFalse(new InstantCommand(() -> s_Claw.setDutyCycle(0)));  // Once we get a piece, hold it
    haveGamePiece.onTrue(new WaitCommand(0.5).andThen(new InstantCommand(() -> s_Claw.setDutyCycle(0)))); // Once we shoot a piece, stop motors
    haveGamePiece.onChange(new InstantCommand(() -> SmartDashboard.putBoolean("lightbreak", haveGamePiece.getAsBoolean())));
    
    /* Uncomment line-by-line as we install: Claw, Elevator, Wrist */
    interruptButton.onTrue(new InstantCommand(() -> {
      s_Claw.setDutyCycle(0);
      s_Elevator.getCurrentCommand().cancel();
      s_Elevator.setDutyCycle(0);
      s_Wrist.getCurrentCommand().cancel();
      s_Wrist.setDutyCycle(0);
    }));
 }

//  public Command getAutonomousCommand() {
//    return autoChooser.getSelected();
//  }

  public void teleopInit(){
    swerve.resetToAbsolute();

    swerve.setDefaultCommand(
      new TeleopSwerve(
        swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis), // To enable the autoaim button again, comment this line and uncomment the line below
        // () -> (autoAimButton.getAsBoolean() ? -s_Limelight.getRobotRotationtoSpeaker()*Preferences.getDouble("AutoAimStrength", 1.0)/100.0 : -driver.getRawAxis(rotationAxis)),
        () -> robotCentric));

    /* Once elevator is installed */
    s_Elevator.setDefaultCommand(
      new ElevatorDefault(s_Elevator,
      () -> -operator.getRawAxis(elevatorAxis))
    );
    /* For testing wrist. I'd like to only control with SetWristCommand during competition */
    s_Wrist.setDefaultCommand(
      new WristDefault(s_Wrist, s_Elevator,
      () -> -operator.getRawAxis(wristAxis))
    );
    
  }

  /*
  public Command scoreCoral(Constants.Level level) {
    if (s_Limelight.updatePose(swerve)) {
      Location reefspot = Location.ReefLeft;
      if (operator.getRawAxis(reefAxis) > 0) { reefspot = Location.ReefRight; }

      Pose2d goalPose = null;
      // TODO - code to decide what pose to end up in.
      // For example, lining up on the right side of blue reef, our pose is at:
      //    {x:6.13, y:4.01, rot:180}       I used pathplanner to find numbers for this pose
      // This pose would be better for grabbing the algae rather than scoring coral, though.
      // 
      switch(s_Limelight.getTagID()) {
        case 1: break;
      }
      
      Command driveCommand = AutoBuilder.pathfindToPose(goalPose, new PathConstraints(2, 4, 3, 6));

      return driveCommand
        .alongWith(new SetElevator(s_Elevator, level))
        .andThen(new WaitCommand(0.0));
    }
    else { return null; }
  }
     */

  public void teleopExit() {
    swerve.removeDefaultCommand();
    s_Elevator.removeDefaultCommand(); // Once elevator is installed
    s_Wrist.removeDefaultCommand(); // Once wrist is installed
  }

  public void autoInit(){
    swerve.resetToAbsolute();
  }

  public void testInit(){
    // Swerve SysID testing. Sets wheels forward and assigns each test to a button.
    swerve.resetToAbsolute();
    swerve.testInit().schedule();

    swerve_quasiF.onTrue( swerve.SysIDQuasiF().until( swerve_quasiF.negate()));
    swerve_quasiR.onTrue( swerve.SysIDQuasiR().until( swerve_quasiR.negate()));
    swerve_dynF.onTrue(   swerve.SysIDDynF().until(   swerve_dynF.negate()));
    swerve_dynR.onTrue(   swerve.SysIDDynR().until(   swerve_dynR.negate()));
    
    // Elevator SysID testing. Sets wheels forward and assigns each test to a button.
    new SetElevator(s_Elevator, Level.L3);  // Raise the elevator to start, so that we can find the breakeven voltage that overcomes gravity.

    elevator_quasiF.onTrue( s_Elevator.SysIDQuasiF().until( swerve_quasiF.negate()));
    elevator_quasiR.onTrue( s_Elevator.SysIDQuasiR().until( swerve_quasiR.negate()));
    elevator_dynF.onTrue(   s_Elevator.SysIDDynF().until(   swerve_dynF.negate()));
    elevator_dynR.onTrue(   s_Elevator.SysIDDynR().until(   swerve_dynR.negate()));
  }

  public void testExit(){
    swerve.getCurrentCommand().cancel();

    // This command clears all button bindings. I think if we switch straight from test mode to
    // teleop, we won't have any buttons... but we'd still have default commands.
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
  }
}