// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SetElevator;
import frc.robot.commands.DefaultCommands.TeleopSwerve;
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
  private final JoystickButton zeroGyro =
  new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton robotCentricBumper =
  new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton resetOdometry = 
  new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton autoAimButton = 
  new JoystickButton(driver, XboxController.Button.kA.value);
  private boolean robotCentric = false;

  /* Operator Buttons */
  private final int reefAxis = XboxController.Axis.kLeftX.value;

  

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Limelight speakerLimelight = new Limelight("limelight");
  private final Elevator s_Elevator = new Elevator();

  /* Robot Variables */
  //private final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    
    configureBindings();

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
        // () -> (autoAimButton.getAsBoolean() ? -speakerLimelight.getRobotRotationtoSpeaker()*Preferences.getDouble("AutoAimStrength", 1.0)/100.0 : -driver.getRawAxis(rotationAxis)),
        () -> robotCentric));
  }

  public Command scoreCoral(Constants.Level level) {
    if (speakerLimelight.updatePose(swerve)) {
      Location reefspot = Location.ReefLeft;
      if (operator.getRawAxis(reefAxis) > 0) { reefspot = Location.ReefRight; }

      Pose2d goalPose = null;
      // TODO - code to decide what pose to end up in.
      // For example, lining up on the right side of blue reef, our pose is at:
      //    {x:6.13, y:4.01, rot:180}       I used pathplanner to find numbers for this pose
      // This pose would be better for grabbing the algae rather than scoring coral, though.
      Command driveCommand = AutoBuilder.pathfindToPose(goalPose, new PathConstraints(2, 4, 5, 10));

      return driveCommand
        .alongWith(new SetElevator(s_Elevator, level))
        .andThen(new WaitCommand(0.0));
    }
    else { return null; }
  }

  public void teleopExit() {
    swerve.removeDefaultCommand();
  }

  public void autoInit(){
    swerve.resetToAbsolute();
  }

  public void testInit(){
    swerve.resetToAbsolute();
    //CommandScheduler.getInstance().schedule(swerve.getTestCommand());
  }
}