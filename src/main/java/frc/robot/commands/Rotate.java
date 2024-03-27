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
  public void initialize()
  {
  }

  @Override
  public void execute() 
  {
    s_Swerve.drive(
      new Translation2d(0, 0),
      angle,
      !robotCentricSup.getAsBoolean(),
      false
    );
  }
}