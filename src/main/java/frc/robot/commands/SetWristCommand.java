// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class SetWristCommand extends Command{
    private Wrist s_Wrist;
    private Double targetAngle;
    private DoubleSupplier JoystickInput;
    private SimpleMotorFeedforward feedForward;

    /**
     * A Command to spin the scoring assembly on the elevator to a specified angle.
     * This command is good for teleop, as it can be interrupted by the operator.
     * 
     * @param s_Wrist The subsystem to control
     * @param targetAngle The angle to spin to. Should be between Constants.Wrist.minimumAngle and Constants.Wrist.maximumAngle
     * @param JoystickInput A link to joystick input, which can interrupt the command. Should be the same joystick for manual control of the subsystem.
     */
    public SetWristCommand(Wrist s_Wrist, Double targetAngle, DoubleSupplier JoystickInput) {
        this.s_Wrist = s_Wrist;
        addRequirements(s_Wrist);
        
        this.targetAngle = targetAngle;
        this.JoystickInput = JoystickInput; // Strictly for interrupting

        feedForward = new SimpleMotorFeedforward(Constants.SVA.WristSVA[0],Constants.SVA.WristSVA[1],Constants.SVA.WristSVA[2]);
    }

    /**
     * A Command to spin the scoring assembly on the elevator to a specified angle.
     * This command is good for autonomous routines, because it doesn't have a joystick input for interrupting.
     * 
     * @param s_Wrist The subsystem to control
     * @param targetAngle The angle to spin to. Should be between Constants.Wrist.minimumAngle and Constants.Wrist.maximumAngle
     */
    public SetWristCommand(Wrist s_Wrist, Double targetAngle) {
        this(s_Wrist, targetAngle, () -> 0.0);
    }
    public SetWristCommand(Wrist s_Wrist, int targetAngle, DoubleSupplier JoystickInput) {
        this(s_Wrist, (double) targetAngle, JoystickInput);
    }
    public SetWristCommand(Wrist s_Wrist, int targetAngle) {
        this(s_Wrist, (double) targetAngle, () -> 0.0);
    }

    @Override
    public void initialize() {
        s_Wrist.setTargetAngle(targetAngle);
    }

    @Override
    public void execute() {
        s_Wrist.doNextVoltage();
    }

    @Override
    public void end(boolean interrupted) {
        s_Wrist.setTargetAngle(s_Wrist.getAngle());
        // s_Wrist.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return (
            Math.abs((s_Wrist.getAngle()) - targetAngle) <= Constants.Wrist.angleTolerance ||
            Math.abs(JoystickInput.getAsDouble()) > .1
        );
    }
}