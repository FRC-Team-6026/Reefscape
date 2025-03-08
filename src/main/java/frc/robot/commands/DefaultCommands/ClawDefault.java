package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class ClawDefault extends Command{
    private Claw s_Claw;
    private DoubleSupplier speedSup;

    public ClawDefault(
        Claw s_Claw,
        DoubleSupplier speedSup
    ) {
        this.s_Claw = s_Claw;
        addRequirements(s_Claw);
        
        this.speedSup = speedSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        double speedPref = Preferences.getDouble("ClawVoltage", 1);

        // Applying deadband so thumbsticks that are slightly off dont trigger command
        double speed = MathUtil.applyDeadband(speedSup.getAsDouble(), 0.1);
        double voltage = speed * speedPref;
        
        if (Math.abs(voltage) < Constants.Electrical.neoMinVoltage)
            s_Claw.setDutyCycle(0);
        else
            s_Claw.setVoltage(voltage);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}