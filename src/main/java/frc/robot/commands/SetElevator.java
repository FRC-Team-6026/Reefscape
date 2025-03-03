package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.Constants.Level;

/*   Todo to finish this commands functionality:
 * Fill out Elevator subsystem      
 *  ☑ fill out constants            Done! Thank you, Blake.
 *  - fill out any functions within the file
 * Fill out this command
 *  ☑ any initialization?           Init profile speed from Preferences
 *  ☑ power motor from execute      Check to make sure it's good, undo completely disabling if outside min/max
 *  ☑ any end commands?             Turns off motor
 *  ☑ set end condition             Cancel button or at target (within tolerance)
 */

public class SetElevator extends Command{
    private Elevator s_Elevator;
    //private Level level;
    private double targetHeight;
    private BooleanSupplier cancelButton = (() -> { return false; });

    public static double retractedHeight = 0;
    public static double processorHeight = 1;
    public static double L1Height = 4,  L2Height = 6, L3Height = 8, L4Height = 10;
    public static double L2AHeight = 7, L3AHeight = 9;
    public static double netHeight = 12;
    // Retracted    = All the way down
    // Processor    = Floor Algae goal
    // L1/L2/L3/L4  = Coral targets
    // L2A/L3A      = Algae pickups
    // Net          = Barge net

    public SimpleMotorFeedforward feedForward;
    private double lastVel;

    /**
     * An uninterruptable command to set the elevator to a specific height
     * Use this constructor for autonomous routines.
     * 
     * @param Elevator the mechanical elevator subsystem
     * @param Level the height to set the elevator to
     */
    public SetElevator(Elevator elevator, Level level) {
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

        feedForward = new SimpleMotorFeedforward(Constants.SVA.ElevSVA[0],Constants.SVA.ElevSVA[1],Constants.SVA.ElevSVA[2]);

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
        double speed = Preferences.getDouble("ElevatorVoltage", 0.2);
        s_Elevator.elevProfiledPID.setConstraints(new TrapezoidProfile.Constraints(speed, speed*2));     // Reach max speed in 0.5s
        lastVel = 0;
    }

    @Override
    public void execute() {
        double attemptVoltage = s_Elevator.elevProfiledPID.calculate(s_Elevator.elevatorEncoder1.getPosition() * 360, targetHeight); // Calculate profiled voltage. Reverse voltage to get correct direction
        TrapezoidProfile.State state = s_Elevator.elevProfiledPID.getSetpoint();
        
        
        SmartDashboard.putNumber("Elevator velocity attempt", state.velocity);
        double FFVoltage = feedForward.calculate(state.velocity, state.velocity - lastVel);
        SmartDashboard.putNumber("Elevator attempt Voltage", attemptVoltage);
        SmartDashboard.putNumber("Elevator FF Voltage", FFVoltage);
        
        // error V       expected V          static voltage (fixed problems last year)                            overcome gravity
        attemptVoltage += FFVoltage + (0.2 * Math.signum(targetHeight - (s_Elevator.elevatorEncoder1.getPosition()))) + Constants.Elevator.gravityConstant;

        lastVel = state.velocity;
        //s_Elevator.lastVoltageAttempt = attemptVoltage;

        // This positional clamping *shouldn't* be neccesary, but it's an extra precaution
        // TODO - this is temporarily set to completely disable if outside of min/max height.
        if (s_Elevator.elevatorEncoder1.getPosition() >= Constants.Elevator.maxHeight)     // if we're at or past maximum, only allow moving back
            attemptVoltage = Math.min(attemptVoltage, 0);
        if (s_Elevator.elevatorEncoder1.getPosition() <= Constants.Elevator.minHeight)     // if we're at or past minimum, only allow moving forawrd
            attemptVoltage = Math.max(attemptVoltage, 0);
            
        s_Elevator.setVoltage(MathUtil.clamp(attemptVoltage, -Constants.Elevator.maxVoltage, Constants.Elevator.maxVoltage));
    }

    @Override
    public void end(boolean interrupted) {
        s_Elevator.setDutyCycle(0);     // Turn off the motor
    }

    @Override
    public boolean isFinished() {
        return (cancelButton.getAsBoolean() ||
               Math.abs(s_Elevator.getHeight() - targetHeight) <= Constants.Elevator.tolerance);
    }
}