package frc.robot.commands.DefaultCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// Useful for testing the wrist. Ideally, we don't even use this
// command, but only set the wrist via the SetWristCommand.

public class WristDefault extends Command{
    private Wrist s_Wrist;
    private DoubleSupplier speedSup;
    public Elevator s_Elevator;

    private Timer wristTimer;

    public WristDefault(
        Wrist s_Wrist,
        Elevator s_Elevator,
        DoubleSupplier speedSup
    ) {
        this.s_Wrist = s_Wrist;
        this.s_Elevator = s_Elevator;
        addRequirements(s_Wrist);

        wristTimer = new Timer();
        
        this.speedSup = speedSup;
    }

    @Override
    public void initialize(){
        wristTimer.reset();
        wristTimer.start();
    }

    @Override
    public void execute(){
        double speedPref = Preferences.getDouble("WristSpeed", 0.2);

        // Applying deadband so thumbsticks that are slightly off dont trigger command
        double speed = MathUtil.applyDeadband(speedSup.getAsDouble(), 0.1);
        
        if (s_Wrist.getAngle() < (s_Elevator.getHeight() > 1 ? Constants.Elevator.selfDestructAngle + 5 : Constants.Wrist.minimumAngle))  // Is our elevator up? if so, dont move the wrist past the self-destruct angle.
            speed = MathUtil.clamp(speed, -Constants.Wrist.maxVoltage, 0);
        if (s_Wrist.getAngle() > Constants.Wrist.maximumAngle)
            speed = MathUtil.clamp(speed, 0, Constants.Wrist.maxVoltage);
        
        s_Wrist.addTargetAngle((speed * Math.min(wristTimer.get(), 1)));
        s_Wrist.doNextVoltage();
        
        wristTimer.reset();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}