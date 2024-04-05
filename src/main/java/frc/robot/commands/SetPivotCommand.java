// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class SetPivotCommand extends Command{
    private Pivot s_Pivot;
    private Double targetAngle;
    private DoubleSupplier JoystickInput;
    private SimpleMotorFeedforward feedForward;

    /**
     * A Command to spin the shooter assembly to a specified angle.
     * @param s_Pivot The subsystem to control
     * @param targetAngle The angle to spin to. Should be between Constants.Pivot.minimumAngle and Constants.Pivot.maximumAngle
     * @param JoystickInput A link to joystick input, which can interrupt the command. Should be the same joystick for manual control of the subsystem.
     */
    
    public SetPivotCommand(Pivot s_Pivot, Double targetAngle, DoubleSupplier JoystickInput) {
        this.s_Pivot = s_Pivot;
        addRequirements(s_Pivot);
        
        this.targetAngle = targetAngle;
        this.JoystickInput = JoystickInput; // Strictly for interrupting

        feedForward = new SimpleMotorFeedforward(Constants.SVA.PivotSVA[0],Constants.SVA.PivotSVA[1],Constants.SVA.PivotSVA[2]);
    }
    public SetPivotCommand(Pivot s_Pivot, Double targetAngle) {
        this(s_Pivot, targetAngle, () -> 0.0);
    }
    public SetPivotCommand(Pivot s_Pivot, int targetAngle, DoubleSupplier JoystickInput) {
        this(s_Pivot, (double)targetAngle, JoystickInput);
    }

    @Override
    public void initialize() {
        s_Pivot.pivotPID.reset(s_Pivot.PivotEncoder.getAbsolutePosition() * 360);
        s_Pivot.isTrackingAngle = true;
        // double attemptVoltage = s_Pivot.pivotPID.calculate(s_Pivot.PivotEncoder.getAbsolutePosition() * 360, targetAngle);
    }

    @Override
    public void execute() {
        double attemptVoltage = s_Pivot.pivotPID.calculate(s_Pivot.PivotEncoder.getAbsolutePosition() * 360, targetAngle); // Calculate profiled voltage. Reverse voltage to get correct direction
        SmartDashboard.putNumber("Pivot velocity attempt", s_Pivot.pivotPID.getSetpoint().velocity);
        double FFVoltage = feedForward.calculate(s_Pivot.pivotPID.getSetpoint().velocity);
        SmartDashboard.putNumber("Pivot attempt Voltage", attemptVoltage);
        SmartDashboard.putNumber("Pivot FF Voltage", FFVoltage);
        
        attemptVoltage += FFVoltage + (0.2 * Math.signum(targetAngle - (s_Pivot.PivotEncoder.getAbsolutePosition() * 360)));

        s_Pivot.lastVoltageAttempt = attemptVoltage;

        // This positional clamping *shouldn't* be neccesary, but it's an extra precaution
        if (s_Pivot.PivotEncoder.getAbsolutePosition() * 360 >= Constants.Pivot.maximumAngle)     // if we're at or past maximum, only allow moving back
            attemptVoltage = Math.min(attemptVoltage, 0);
        if (s_Pivot.PivotEncoder.getAbsolutePosition() * 360 <= Constants.Pivot.minimumAngle)     // if we're at or past minimum, only allow moving forawrd
            attemptVoltage = Math.max(attemptVoltage, 0);
            
        s_Pivot.PivotMotor.spark.setVoltage(MathUtil.clamp(attemptVoltage, -Constants.Pivot.maxVoltage, Constants.Pivot.maxVoltage));
    }

    @Override
    public void end(boolean interrupted) {
        s_Pivot.isTrackingAngle = false;
        s_Pivot.PivotMotor.spark.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return (
            Math.abs((s_Pivot.PivotEncoder.getAbsolutePosition() * 360) - targetAngle) <= Constants.Pivot.angleTolerance ||
            Math.abs(JoystickInput.getAsDouble()) > .1
        );
    }
}

