package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDefault extends Command{
    private Elevator s_Elevator;
    private DoubleSupplier speedSup;

    public ElevatorDefault(
        Elevator s_Elevator,
        DoubleSupplier speedSup
    ) {
        this.s_Elevator = s_Elevator;
        addRequirements(s_Elevator);
        
        this.speedSup = speedSup;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double speedPref = Preferences.getDouble("ElevatorSpeed", 1.0);

        // Applying deadband so thumbsticks that are slightly off dont trigger command
        double speed = MathUtil.applyDeadband(speedSup.getAsDouble(), 0.1);
        speed += (speed > 0 ? -0.1 : 0.1);  // Put minimum value back down to 0 after deadband.
        s_Elevator.setVoltage(speedSup.getAsDouble() * speedPref);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}