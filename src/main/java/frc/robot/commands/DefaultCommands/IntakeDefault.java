package frc.robot.commands.DefaultCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends Command{
    private Intake s_Intake;
    private BooleanSupplier activeSup;
    private DoubleSupplier speedSup;
    private BooleanSupplier reverseIntake;

    public IntakeDefault(
        Intake s_Intake,
        BooleanSupplier activeSup,
        DoubleSupplier speedSup,
        BooleanSupplier reverseIntake) {

        this.s_Intake = s_Intake;
        addRequirements(s_Intake);
        //bob
        // ^ This is Alex's legacy. It will live on forever.
        this.activeSup = activeSup;
        this.speedSup = speedSup;
        this.reverseIntake = reverseIntake;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Reverse Intake", reverseIntake.getAsBoolean());

        if(activeSup.getAsBoolean() == true){
            s_Intake.setVelocity(speedSup.getAsDouble()*(reverseIntake.getAsBoolean() ? -1 : 1));
        } else {
            s_Intake.setDutyCylce(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
