package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends Command{
    private Intake s_Intake;
    private BooleanSupplier activeSup;
    private DoubleSupplier speedSup;

    public IntakeDefault(
        Intake s_Intake,
        BooleanSupplier activeSup,
        DoubleSupplier speedSup) {

    this.s_Intake = s_Intake;
    addRequirements(s_Intake);
    
    this.activeSup = activeSup;
    this.speedSup = speedSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(activeSup.getAsBoolean() == true){

        } else {

        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
