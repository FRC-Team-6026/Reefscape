package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;

/*
 * Considering deleting this whole file, and constructing scoring commands from
 * component commands in RobotContainer. I'm leaving the file for now.
 */



public class ScoreCoral extends Command {
    private Elevator s_Elevator;
    private Limelight s_Limelight;
    private Level level;
    private BooleanSupplier cancelButton;

    public static enum Level { L1, L2, L3, L4 }

    /**
     * A uninterruptable command to score a coral piece on a specified level.
     * Use this constructor for autonomous routines.
     * 
     * @param Elevator the mechanical elevator subsystem
     * @param limelight the limelight camera subsystem
     * @param Level where to score the coral. L1 is the trough, L4 is the top pipe
     */
    public ScoreCoral(Elevator elevator, Limelight limelight, Level L) {
        this(elevator, limelight, L, null);
        cancelButton = (() -> {return false;});
    }

    /**
     * A command to score a coral piece on a specified level.
     * This command is good for the teleop phase.
     * 
     * @param Elevator the mechanical elevator subsystem
     * @param limelight the limelight camera subsystem
     * @param Level where to score the coral. L1 is the trough, L4 is the top pipe
     * @param cancelButton a button that will interrupt the command
     */
    public ScoreCoral(Elevator elevator, Limelight limelight, Level L, BooleanSupplier cancelButton) {
        this.s_Elevator = elevator;
        this.s_Limelight = limelight;
        this.level = L;
        this.cancelButton = cancelButton;
        addRequirements(elevator);
    }


}