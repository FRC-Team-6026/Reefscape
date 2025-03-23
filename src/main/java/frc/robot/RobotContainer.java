// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SetElevatorPos;
import frc.robot.commands.SetWristPos;
import frc.robot.commands.DefaultCommands.ElevatorDefault;
import frc.robot.commands.DefaultCommands.TeleopSwerve;
import frc.robot.commands.DefaultCommands.WristDefault;
import frc.robot.Constants.Level;
import frc.robot.Constants.Location;

public class RobotContainer {

  public boolean angleConfigured = false;

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
  private final JoystickButton robotCentricButton =
  new JoystickButton(driver, XboxController.Button.kStart.value);
  /** Driver - Y (X on our controller) */
  private final JoystickButton resetOdometry = 
  new JoystickButton(driver, XboxController.Button.kY.value);
  /** Driver - X (Y on our controller) */
  private final JoystickButton alignReefLeftButton = 
  new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton limelightUpdateButton = 
  new JoystickButton(driver, XboxController.Button.kA.value);
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
  /** Operator - Left Stick Y */
  private final int wristAxis = XboxController.Axis.kLeftY.value;
  /** Operator - Right Stick Y */
  private final int elevatorAxis = XboxController.Axis.kRightY.value;
  
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
  /** Operator - Minus Button */
  private final JoystickButton clawReverseButton =
  new JoystickButton(operator, XboxController.Button.kBack.value);
  /** Operator - Right Bumper */
  private final JoystickButton algaeCoralToggle =
  new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  /** Operator - Left Trigger */
  private final int algaeTrigger = XboxController.Axis.kLeftTrigger.value;
  private final Trigger clawReverse = new Trigger(() -> operator.getRawAxis(algaeTrigger) > 0.1);
  /** Operator - Right Trigger */
  private final int instakeTrigger = XboxController.Axis.kRightTrigger.value;
  private final Trigger clawIntake = new Trigger(() -> operator.getRawAxis(instakeTrigger) > 0.1);

  /* Subsystems */
  private final DigitalInput beambreak = new DigitalInput(Constants.Setup.beambreakID);


  // If we add a physical switch for algae detection, uncomment these, and comment the other haveGamePiece trigger
  // private final DigitalInput physicalSwitch = new DigitalInput(Constants.Setup.physicalSwitchID);
  // private final Trigger haveGamePiece = new Trigger(() -> beambreak.get()).or(() -> physicalSwitch.get());
  /** This is returns TRUE if we DO have coral. We are now negating the beambreak to get this. */
  private final Trigger haveGamePiece = new Trigger(() -> !beambreak.get());

  private final Swerve swerve = new Swerve();
  private final Limelight s_Limelight = new Limelight("limelight");
  private final Wrist s_Wrist = new Wrist();
  private final Claw s_Claw = new Claw();
  private final Elevator s_Elevator = new Elevator(s_Wrist);

  /* Robot Variables */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Elevator Commands
    NamedCommands.registerCommand("Elevator - L1", new SetElevatorPos(s_Elevator, Level.L1));
    NamedCommands.registerCommand("Elevator - L2", new SetElevatorPos(s_Elevator, Level.L2));
    NamedCommands.registerCommand("Elevator - L3", new SetElevatorPos(s_Elevator, Level.L3));
    NamedCommands.registerCommand("Elevator - L4", new SetElevatorPos(s_Elevator, Level.L4));
    NamedCommands.registerCommand("Elevator - Retracted", new SetElevatorPos(s_Elevator, Level.Retracted));
    NamedCommands.registerCommand("Elevator - Processor", new SetElevatorPos(s_Elevator, Level.Processor));
    NamedCommands.registerCommand("Elevator - L2 Algae", new SetElevatorPos(s_Elevator, Level.L2A));
    NamedCommands.registerCommand("Elevator - L3 Algae", new SetElevatorPos(s_Elevator, Level.L3A));

    // Wrist Commands
    NamedCommands.registerCommand("Wrist - Intake", new SetWristPos(s_Wrist, Constants.Wrist.minimumAngle));
    NamedCommands.registerCommand("Wrist - L1", new SetWristPos(s_Wrist, Constants.Wrist.L1ScoringAngle));
    NamedCommands.registerCommand("Wrist - L2 / L3", new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle));
    NamedCommands.registerCommand("Wrist - L4", new SetWristPos(s_Wrist, Constants.Wrist.L4ScoringAngle));
    NamedCommands.registerCommand("Wrist - Algae", new SetWristPos(s_Wrist, Constants.Wrist.algaeAngle));

    // Control Commands
    NamedCommands.registerCommand("Claw - Intake", new InstantCommand(() -> 
        s_Claw.setVoltage(-Preferences.getDouble("ClawSpeed", 0.2))
    ));
    NamedCommands.registerCommand("Claw - Reverse", new InstantCommand(() -> 
        s_Claw.setVoltage(Preferences.getDouble("ClawSpeed", 0.2))
    ));
    NamedCommands.registerCommand("Claw - Stop", new InstantCommand(() -> 
        s_Claw.setDutyCycle(0)
    ));

    NamedCommands.registerCommand("Limelight - Init Rotation", new InstantCommand(() -> {s_Limelight.configRotation(swerve.getPose().getRotation().getDegrees() - swerve.getGyro().getYaw());}));
    NamedCommands.registerCommand("Limelight - Config Rotation", new InstantCommand(() -> {angleConfigured = s_Limelight.configRotation(swerve);}).repeatedly().until(() -> angleConfigured));
    NamedCommands.registerCommand("Limelight - Update Pose", new InstantCommand(() -> s_Limelight.updatePose(swerve)));


    s_Wrist.s_Elevator = s_Elevator;

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
    if (!Preferences.containsKey("WristKV")) {
      Preferences.initDouble("WristKV", 0.0);
    }

    swerve.zeroGyro();

    /**
     * Create and populate a sendable chooser with all PathPlannerAutos in the project
     * This section of code is copied from AutoBuilder.buildAutoChooser and modified to fit what we want.
     */

    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }
    else {
      // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
      // SmartDashboard.putData("Auto Mode", autoChooser);
      
      
      autoChooser = new SendableChooser<Command>();

      // SendableChooser<Command> chooser = new SendableChooser<Command>();
      List<String> autoNames = AutoBuilder.getAllAutoNames();
      PathPlannerAuto defaultOption = null;
      List<PathPlannerAuto> options = new ArrayList<>();

      String defaultAutoName = Constants.AutoConfig.defaultAutoName();
      String[] ignoreAutos = Constants.AutoConfig.supportAutoList();
      for (String ignore : ignoreAutos) { autoNames.remove(ignore); }

      for (String autoName : autoNames) {
        PathPlannerAuto auto = new PathPlannerAuto(autoName);

        if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName))
                  { defaultOption = auto; }
        else      { options.add(auto); }
      }

      if (defaultOption == null) {
        autoChooser.setDefaultOption("None", Commands.none());
      } else {
        autoChooser.setDefaultOption(defaultOption.getName(), defaultOption);
        autoChooser.addOption("None", Commands.none());
      }

      for (PathPlannerAuto auto : options ) 
        {autoChooser.addOption(auto.getName(), auto);}
      /* Compound Auto Routines */
      /* Example:
      PathPlannerAuto getCoral = new PathPlannerAuto("DriveToCoralStation");
      PathPlannerAuto scoreCoralL3 = new PathPlannerAuto("DriveToReef");
      chooser.addOption("Get and Score Coral", getCoral.andThen(scoreCoralL3.onlyWhile(hasCoral)));
      */

      SmartDashboard.putData("Auto Mode", autoChooser);
      
      s_Limelight.configRotation(swerve);
    }
  }
  private void configureBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    robotCentricButton.onTrue(new InstantCommand(() -> {
      robotCentric = !robotCentric;
      SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
    }));

    resetOdometry.onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));

    // alignReefLeftButton.onTrue(driveToReef(Location.ReefLeft).until(alignReefLeftButton.negate()));
    limelightUpdateButton.onTrue(new InstantCommand(() -> {
      Pose2d data = s_Limelight.getRobotPoseInTargetSpace();
      SmartDashboard.putString("AprilTag Targeting", "("+
        Math.round(data.getX()*100)/100.0+", "+
        Math.round(data.getY()*100)/100.0+", "+
        Math.round(data.getRotation().getDegrees()));
    }));
    // limelightUpdateButton.onTrue(Commands.run(() -> {
    //   Pose2d data = s_Limelight.getRobotPoseInTargetSpace();
    //   SmartDashboard.putString("AprilTag Targeting", "("+data.getX()+", "+data.getY() + ", "+data.getRotation());
    // }).until(limelightUpdateButton.negate()));
    /* Operator Buttons */
    /* Once claw is installed: */
    clawReverse.onTrue(new InstantCommand(() -> s_Claw.setVoltage(Preferences.getDouble("ClawSpeed", 0.2))));
    clawIntake.onTrue(new InstantCommand(() -> s_Claw.setVoltage(-Preferences.getDouble("ClawSpeed", 0.2))));

    // TODO - possibly smarter position controls? For instance, a conditional command
    // to check if our elevator is up on this first command, simply setting the wrist
    // to the minimum angle if it's not.

    // All of our position preset commands.
    // If the algae toggle is not held:
    //  move the wrist to a safe position, then drop the elevator, then move it to intake position
    // If the algae button is held:
    //  move the wrist and elevator at the same time to the processor position
    elevFloorButton.onTrue(new ConditionalCommand(
      new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle).andThen(
      new SetElevatorPos(s_Elevator, Constants.Level.Retracted).andThen(
      new SetWristPos(s_Wrist, Constants.Wrist.minimumAngle)))
      ,
      new SetElevatorPos(s_Elevator, Constants.Level.Processor).alongWith(
      new SetWristPos(s_Wrist, Constants.Wrist.algaeAngle)),
      algaeCoralToggle.negate()).until(interruptButton));
    elevL2Button.onTrue(new ConditionalCommand(
      new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle).andThen(
      new SetElevatorPos(s_Elevator, Constants.Level.L2))
      ,
      new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle).andThen(
      new SetElevatorPos(s_Elevator, Constants.Level.L2A).andThen(
      new SetWristPos(s_Wrist, Constants.Wrist.algaeAngle))),
      algaeCoralToggle.negate()).until(interruptButton));
    elevL3Button.onTrue(new ConditionalCommand(
      new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle).andThen(
      new SetElevatorPos(s_Elevator, Constants.Level.L3))
      ,
      new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle).andThen(
      new SetElevatorPos(s_Elevator, Constants.Level.L3A).andThen(
      new SetWristPos(s_Wrist, Constants.Wrist.algaeAngle))),
      algaeCoralToggle.negate()).until(interruptButton));
    elevL4Button.onTrue(new ConditionalCommand(
      new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle).andThen(
      new SetElevatorPos(s_Elevator, Constants.Level.L4)).andThen(
      new SetWristPos(s_Wrist, Constants.Wrist.L4ScoringAngle))
      ,
      new SetWristPos(s_Wrist, 150.0).andThen(
      new SetElevatorPos(s_Elevator, Constants.Level.L4).andThen(
      new SetWristPos(s_Wrist, Constants.Wrist.bargeAngle))),
      algaeCoralToggle.negate()).until(interruptButton));
    
    clawReverseButton.onTrue(new InstantCommand(() -> s_Claw.setVoltage(0.5)));
    clawReverseButton.onFalse(new InstantCommand(() -> s_Claw.setDutyCycle(0.0)));
    
    /* Once beambreak is installed */
    haveGamePiece.onTrue(new InstantCommand(() -> s_Claw.setDutyCycle(0))
            .andThen(new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle)));
    haveGamePiece.onFalse(new WaitCommand(0.5).andThen(new InstantCommand(() -> s_Claw.setDutyCycle(0)))
            .andThen(new SetWristPos(s_Wrist, Constants.Wrist.L23ScoringAngle)));
    haveGamePiece.onChange(new InstantCommand(() -> SmartDashboard.putBoolean("lightbreak", haveGamePiece.getAsBoolean())));
   
    /* Uncomment line-by-line as we install: Claw, Elevator, Wrist */
    interruptButton.onTrue(new InstantCommand(() -> {
      s_Claw.setDutyCycle(0);
      s_Wrist.setAngle(s_Wrist.getAngle());
    }));
 }

 public Command getAutonomousCommand() {
   return autoChooser.getSelected();
 }

  public void teleopInit() {
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

  public Command driveToReef(Location reefspot) {
    if (s_Limelight.updatePose(swerve)) {
      //if (operator.getRawAxis(reefAxis) > 0) { reefspot = Location.ReefRight; }

      Pose2d goalPose = null;
      // TODO - code to decide what pose to end up in.
      // For example, lining up on the right side of blue reef, our pose is at:
      //    {x:5.8, y:4.02, rot:180}       I used pathplanner to find numbers for this pose
      // This pose would be better for grabbing the algae rather than scoring coral, though.
      // 
      // Right now, we're only looking at the left coral branch. 

      // if (reefspot == Location.ReefLeft)
      int tag = s_Limelight.getTagID();
      SmartDashboard.putNumber("Found tag ID", tag);
      switch(tag) {
        case  6: goalPose = new Pose2d(13.57, 2.82, Rotation2d.fromDegrees(120)); break;
        case  7: goalPose = new Pose2d(14.37, 3.86, Rotation2d.k180deg); break;
        case  8: goalPose = new Pose2d(13.85, 5.08, Rotation2d.fromDegrees(-120)); break;
        case  9: goalPose = new Pose2d(12.54, 5.24, Rotation2d.fromDegrees(-60)); break;
        case 10: goalPose = new Pose2d(11.76, 4.19, Rotation2d.kZero); break;
        case 11: goalPose = new Pose2d(12.27, 2.97, Rotation2d.fromDegrees(60)); break;
        case 17: goalPose = new Pose2d(3.69, 2.97, Rotation2d.fromDegrees(60)); break;
        case 18: goalPose = new Pose2d(3.19, 4.19, Rotation2d.kZero); break;
        case 19: goalPose = new Pose2d(3.98, 5.24, Rotation2d.fromDegrees(-60)); break;
        case 20: goalPose = new Pose2d(5.28, 5.08, Rotation2d.fromDegrees(-120)); break;
        case 21: goalPose = new Pose2d(5.80, 3.86, Rotation2d.k180deg); break;
        case 22: goalPose = new Pose2d(5.00, 2.82, Rotation2d.fromDegrees(120)); break;
      }
      if (goalPose != null) 
        return AutoBuilder.pathfindToPose(goalPose, new PathConstraints(2, 4, 3, 6));
      else 
        return new InstantCommand(() -> {});
    }
    else
      return new InstantCommand(() -> {});
  }

  public void teleopExit() {
    swerve.removeDefaultCommand();
    s_Elevator.removeDefaultCommand(); // Once elevator is installed
    s_Wrist.removeDefaultCommand(); // Once wrist is installed
  }

  public void autoInit() {
    swerve.resetToAbsolute();
  }

  public void testInit() {
    // Swerve SysID testing. Sets wheels forward and assigns each test to a button.
    swerve.resetToAbsolute();
    swerve.testInit().schedule();

    swerve_quasiF.onTrue( swerve.SysIDQuasiF().until( swerve_quasiF.negate()));
    swerve_quasiR.onTrue( swerve.SysIDQuasiR().until( swerve_quasiR.negate()));
    swerve_dynF.onTrue(   swerve.SysIDDynF().until(   swerve_dynF.negate()));
    swerve_dynR.onTrue(   swerve.SysIDDynR().until(   swerve_dynR.negate()));
    
    // Elevator SysID testing.

    // Raise the elevator to start, so that we can find the breakeven voltage that overcomes gravity.
    elevator_quasiF.onTrue( new SetElevatorPos(s_Elevator, Level.L2).andThen(
                            new WaitCommand(0.1).andThen(
                            s_Elevator.SysIDQuasiF().until( swerve_quasiF.negate()))));
    elevator_quasiR.onTrue( s_Elevator.SysIDQuasiR().until( swerve_quasiR.negate()));
    elevator_dynF.onTrue(   s_Elevator.SysIDDynF().until(   swerve_dynF.negate()));
    elevator_dynR.onTrue(   s_Elevator.SysIDDynR().until(   swerve_dynR.negate()));
  }

  public void testExit() {
    swerve.getCurrentCommand().cancel();

    // This command clears all button bindings. I think if we switch straight from test mode to
    // teleop, we won't have any buttons... but we'd still have default commands.
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
  }
}