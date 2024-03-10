package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class PivotDefault extends Command{
    private Pivot s_Pivot;
    private DoubleSupplier inputSup;

    public PivotDefault(
        Pivot s_Pivot,
        DoubleSupplier angleSup
    ) {
        this.s_Pivot = s_Pivot;
        addRequirements(s_Pivot);
        
        this.inputSup = angleSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double input = inputSup.getAsDouble();
        if (s_Pivot.PivotEncoder.getAbsolutePosition() * 360 >= Constants.Pivot.maximumAngle)     // if we're at or past maximum, only allow moving back
            input = Math.min(input, 0);
        if (s_Pivot.PivotEncoder.getAbsolutePosition() * 360 <= Constants.Pivot.minimumAngle)     // if we're at or past minimum, only allow moving forawrd
            input = Math.max(input, 0);
        s_Pivot.lastVoltageAttempt = input;
        s_Pivot.PivotMotor.spark.setVoltage(input * Constants.Pivot.maxVoltage/2);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
