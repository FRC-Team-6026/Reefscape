package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

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
    // private Level level;
    private double level;
    private BooleanSupplier cancelButton = (() -> { return false; });

    // Keep as an enum, or split out to separate height doubles?
    //public static enum Level {Retracted, Processor, L1, L2, L2A, L3, L3A, L4}
    public static double retracted = 0;
    public static double processor = 1;
    public static double L1 = 4;
    public static double L2 = 6;
    public static double L3 = 8;
    public static double L4 = 10;
    public static double L2A = 7;
    public static double L3A = 9;
    public static double net = 12;
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
    public SetElevator(Elevator elevator, double level) {
        this.s_Elevator = elevator;
        this.level = level;
        addRequirements(s_Elevator);
    }
    /**
     * A command to set the elevator to a specific height
     * This command is good for the teleop phase.
     * 
     * @param Elevator the mechanical elevator subsystem
     * @param Level the height to set the elevator to
     */
    public SetElevator(Elevator elevator, double level, BooleanSupplier cancelButton) {
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
