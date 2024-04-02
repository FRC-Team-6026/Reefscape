package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class Rotate extends Command {
  private Swerve s_Swerve;
  private TrapezoidProfile angleProfile;
  private double anglePosition = 0.0;
  private double angleVelocity = 0.0;
  private Limelight limelight;

  public Rotate(
      Swerve s_Swerve,
      Limelight limelight) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    angleProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.25, 0.5)); // TODO - Figure this out (DONT RUN THE ROBOT)
  }

  @Override
  public void execute() {
    TrapezoidProfile.State angle = angleProfile.calculate(0.02, new TrapezoidProfile.State(0.0, angleVelocity), new TrapezoidProfile.State(limelight.getRobotRotationtoSpeaker(), 0.0));

    anglePosition = angle.position;
    angleVelocity = angle.velocity;

    s_Swerve.drive(
      new Translation2d(0, 0),
      angleVelocity,
      false,
      false
    );
  }
  
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(
      new Translation2d(0, 0),
      0,
      false,
      false
    );
  }

  @Override
  public boolean isFinished() {
    return Math.abs(limelight.getRobotRotationtoSpeaker()) <= Constants.Swerve.autoAimTolerance;
  }
}