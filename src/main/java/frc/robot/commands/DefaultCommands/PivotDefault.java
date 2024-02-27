package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotDefault extends Command{
    private Pivot s_Pivot;
    private BooleanSupplier button1Pressed;
    private BooleanSupplier button2Pressed;
    private BooleanSupplier button3Pressed;
    private DoubleSupplier angleSup;

    public PivotDefault(
        Pivot s_Pivot,
        BooleanSupplier button1Pressed,
        BooleanSupplier button2Pressed,
        BooleanSupplier button3Pressed,
        DoubleSupplier angleSup
    ) {
        this.s_Pivot = s_Pivot;
        addRequirements(s_Pivot);
        
        this.angleSup = angleSup;
        this.button1Pressed = button1Pressed;
        this.button2Pressed = button2Pressed;
        this.button3Pressed = button3Pressed;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(button1Pressed.getAsBoolean() == true || button2Pressed.getAsBoolean() == true || button3Pressed.getAsBoolean() == true){
            s_Pivot.setVelocity(angleSup.getAsDouble());
        } else {
            s_Pivot.setDutyCylce(0); // idk man
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
