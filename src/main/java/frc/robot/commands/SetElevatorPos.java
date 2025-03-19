package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.Constants.Level;

/** Testing doing straight PID and position control with the elevator. Should
 * be much simpler. Once we know this works, we'll replace SetElevator with
 * this command.  
 */

public class SetElevatorPos extends Command{
    private Elevator s_Elevator;
    //private Level level;
    private double targetHeight;
    private BooleanSupplier cancelButton = (() -> { return false; });
    private boolean isPositionSet;
    double gravityPref;

    public static double retractedHeight = 0.1;
    public static double processorHeight = 1; // Test
    public static double L1Height = retractedHeight,  L2Height = 12.7, L3Height = 26.5, L4Height = 53.0; // 40?
    public static double L2AHeight = 16.5, L3AHeight = 36.5; // Test
    public static double netHeight = 54; // Test
    // Retracted    = All the way down
    // Processor    = Floor Algae goal
    // L1/L2/L3/L4  = Coral targets
    // L2A/L3A      = Algae pickups
    // Net          = Barge net

    /**
     * An uninterruptable command to set the elevator to a specific height
     * Use this constructor for autonomous routines.
     * 
     * @param Elevator the mechanical elevator subsystem
     * @param Level the height to set the elevator to
     */
    public SetElevatorPos(Elevator elevator, Level level) {
        this.s_Elevator = elevator;
        //this.level = level;
        switch (level) {
            case Retracted: this.targetHeight = retractedHeight; break;
            case Processor: this.targetHeight = processorHeight; break;
            case L1:        this.targetHeight = L1Height; break;
            case L2:        this.targetHeight = L2Height; break;
            case L3:        this.targetHeight = L3Height; break;
            case L4:        this.targetHeight = L4Height; break;
            case L2A:       this.targetHeight = L2AHeight; break;
            case L3A:       this.targetHeight = L3AHeight; break;
        }

        addRequirements(s_Elevator);
    }
    /**
     * A command to set the elevator to a specific height
     * This command is good for the teleop phase.
     * 
     * @param Elevator the mechanical elevator subsystem
     * @param Level the height to set the elevator to
     */
    public SetElevatorPos(Elevator elevator, Level level, BooleanSupplier cancelButton) {
        this(elevator, level);
        this.cancelButton = cancelButton;
    }

    @Override
    public void initialize() {
        // double speed = Preferences.getDouble("ElevatorVoltage", 1);
        gravityPref = 0.4;
        if (s_Elevator.wrist.getAngle() > Constants.Elevator.selfDestructAngle) {
            s_Elevator.elevatorController1.setReference(targetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityPref);
            isPositionSet = true;
        }
        else 
            isPositionSet = false;

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Elev Target Height", targetHeight);
        SmartDashboard.putBoolean("Elev Is Position Set", isPositionSet);
        if (!isPositionSet && s_Elevator.wrist.getAngle() > Constants.Elevator.selfDestructAngle) {
            s_Elevator.elevatorController1.setReference(targetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityPref);
            isPositionSet = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // s_Elevator.setDutyCycle(0);     // Turn off the motor
    }

    @Override
    public boolean isFinished() {
        return (cancelButton.getAsBoolean() || Math.abs(s_Elevator.getHeight() - targetHeight) <= Constants.Elevator.tolerance);
    }
}