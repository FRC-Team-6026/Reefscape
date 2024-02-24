package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterWheels;

public class ShooterDefault extends Command{
    private ShooterWheels s_Shooter;
    private BooleanSupplier activeSup;
    private DoubleSupplier speedSup;

    public ShooterDefault(
        ShooterWheels s_Shooter,
        BooleanSupplier activeSup,
        DoubleSupplier speedSup) {

    this.s_Shooter = s_Shooter;
    addRequirements(s_Shooter);
    
    this.activeSup = activeSup;
    this.speedSup = speedSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(activeSup.getAsBoolean() == true){
            s_Shooter.setVelocity(speedSup.getAsDouble());
        } else {
            s_Shooter.setDutyCylce(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
