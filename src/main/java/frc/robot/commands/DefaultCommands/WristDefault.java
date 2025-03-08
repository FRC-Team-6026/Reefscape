package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

// Useful for testing the wrist. Ideally, we don't even use this
// command, but only set the wrist via the SetWristCommand.

public class WristDefault extends Command{
    private Wrist s_Wrist;
    private DoubleSupplier speedSup;

    public WristDefault(
        Wrist s_Wrist,
        DoubleSupplier speedSup
    ) {
        this.s_Wrist = s_Wrist;
        addRequirements(s_Wrist);
        
        this.speedSup = speedSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double speedPref = Preferences.getDouble("WristVoltage", 0.2);

        // Applying deadband so thumbsticks that are slightly off dont trigger command
        double speed = MathUtil.applyDeadband(speedSup.getAsDouble(), 0.1);
        double voltage = speed * speedPref;
        
        if (Math.abs(voltage) < Constants.Electrical.neoMinVoltage)
            s_Wrist.setDutyCycle(0);
        else
            s_Wrist.setVoltage(voltage);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}