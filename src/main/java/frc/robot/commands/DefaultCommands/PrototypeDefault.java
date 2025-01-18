package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Prototype;

public class PrototypeDefault extends Command{
    private Prototype s_Prototype;
    private BooleanSupplier activeSup;
    private DoubleSupplier speedSup;

    public PrototypeDefault(
        Prototype s_Prototype,
        BooleanSupplier activeSup,
        DoubleSupplier speedSup) {

    this.s_Prototype = s_Prototype;
    addRequirements(s_Prototype);
    
    this.activeSup = activeSup;
    this.speedSup = speedSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(activeSup.getAsBoolean() == true){
        s_Prototype.setVoltage(speedSup.getAsDouble());
        } else {
            s_Prototype.setDutyCycle(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}