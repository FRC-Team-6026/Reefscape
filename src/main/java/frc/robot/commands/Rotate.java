package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;

public class Rotate extends Command {
  private Swerve s_Swerve;
  private double angle;
  private BooleanSupplier robotCentricSup;

  public Rotate(
      Swerve s_Swerve,
      double angle,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.angle = angle;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void initialize() {
    // Anything to do when this command starts?
  }

  @Override
  public void execute() {
    s_Swerve.drive(
      new Translation2d(0, 0),
      angle,                              // <-- this expects a value between +/- Constants.Swerve.maxAngularVelocity. If we feed this more that 60, it will rotate right(?) at maximum velocity
      !robotCentricSup.getAsBoolean(),    // <-- This tells the robot to use whatever coordinate system is currently selected by the driver. This should only matter for translation though, so I think we can just make this false.
      false
    );
  }
  
  @Override
  public void end(boolean interrupted) {
    // Anything to do when this command is done?
  }

  @Override
  public boolean isFinished() {
    // What determines when this command is done? Replace the below line, which ends the command instantly
    return true;
  }
}