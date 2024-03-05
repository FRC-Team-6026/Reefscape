package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotDefault extends Command{
    private Pivot s_Pivot;
    private DoubleSupplier angleSup;

    public PivotDefault(
        Pivot s_Pivot,
        DoubleSupplier angleSup
    ) {
        this.s_Pivot = s_Pivot;
        addRequirements(s_Pivot);
        
        this.angleSup = angleSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        s_Pivot.setAngle(angleSup.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
