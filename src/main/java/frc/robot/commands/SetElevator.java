package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.Constants.Level;

/*   Todo to finish this commands functionality:
 * Fill out Elevator subsystem
 *  - fill out constants
 *  - fill out any functions within the file
 * Fill out this command
 *  - any initialization?
 *  - power motor from execute
 *  - any end commands?
 *  - set end condition, including cancel button
 */

public class SetElevator extends Command{
    private Elevator s_Elevator;
    private Level level;
    private double height;
    private BooleanSupplier cancelButton = (() -> { return false; });

    public static double retractedHeight = 0;
    public static double processorHeight = 1;
    public static double L1Height = 4;
    public static double L2Height = 6;
    public static double L3Height = 8;
    public static double L4Height = 10;
    public static double L2AHeight = 7;
    public static double L3AHeight = 9;
    public static double netHeight = 12;
    // Retracted    = all the way down
    // Processor    = the algae goal
    // L1/L2/L3/L4  = Coral targets
    // L2A/L3A      = Algae pickups

    /**
     * An uninterruptable command to set the elevator to a specific height
     * Use this constructor for autonomous routines.
     * 
     * @param Elevator the mechanical elevator subsystem
     * @param Level the height to set the elevator to
     */
    public SetElevator(Elevator elevator, Level level) {
        this.s_Elevator = elevator;
        this.level = level;
        switch (level) {
            case Retracted: this.height = retractedHeight; break;
            case Processor: this.height = processorHeight; break;
            case L1:        this.height = L1Height; break;
            case L2:        this.height = L2Height; break;
            case L3:        this.height = L3Height; break;
            case L4:        this.height = L4Height; break;
            case L2A:       this.height = L2AHeight; break;
            case L3A:       this.height = L3AHeight; break;
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
    public SetElevator(Elevator elevator, Level level, BooleanSupplier cancelButton) {
        this(elevator, level);
        this.cancelButton = cancelButton;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}