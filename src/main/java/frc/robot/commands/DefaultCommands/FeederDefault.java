package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeederDefault extends Command{
    private Feeder s_Feeder;
    private BooleanSupplier activeSup;
    private DoubleSupplier speedSup;

    public FeederDefault(
        Feeder s_Feeder,
        BooleanSupplier activeSup,
        DoubleSupplier speedSup) {

    this.s_Feeder = s_Feeder;
    addRequirements(s_Feeder);
    
    this.activeSup = activeSup;
    this.speedSup = speedSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(activeSup.getAsBoolean() == true){
            s_Feeder.setVoltage(speedSup.getAsDouble());
        } else {
            s_Feeder.setDutyCylce(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
