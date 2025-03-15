package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    public void execute() {
        double speedPref = Preferences.getDouble("ElevatorVoltage", 1);
        double gravityPref = Preferences.getDouble("ElevatorGravity", 0.3);

        // Applying deadband so thumbsticks that are slightly off dont trigger command
        double speed = MathUtil.applyDeadband(speedSup.getAsDouble(), 0.1);
        if (speed < 0.0)
            speed /= 2.0;
        double voltage = 
            speed * speedPref + gravityPref;           // Regular speed setting
            // + MathUtil.clamp(s_Elevator.getHeight()/5.0, 0.0, gravityPref);    // positive element to offset gravity, disabled when elevator is resting

        if (s_Elevator.elevatorEncoder1.getPosition() >= Constants.Elevator.maxHeight)     // if we're at or past maximum, only allow moving back
            voltage = Math.min(voltage, 0);
        if (s_Elevator.elevatorEncoder1.getPosition() <= Constants.Elevator.minHeight)     // if we're at or past minimum, only allow moving forawrd
            voltage = Math.max(voltage, 0);
            
        if (Math.abs(voltage) < Constants.Electrical.neoMinVoltage) {
            SmartDashboard.putNumber("Elevator passed Voltage", 0);
            s_Elevator.setDutyCycle(0);
        }
        else {
            SmartDashboard.putNumber("Elevator passed Voltage", voltage);
            s_Elevator.setVoltage(voltage);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}