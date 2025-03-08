package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    public SparkController elevatorSpark1;
    public SparkController elevatorSpark2;
    
    public RelativeEncoder elevatorEncoder1;
    public RelativeEncoder elevatorEncoder2;

    public SparkClosedLoopController elevatorController1;
    public SparkClosedLoopController elevatorController2;

    public ProfiledPIDController elevProfiledPID;

    public Wrist wrist;

    private SysIdRoutine sysIdRoutine;

    public Elevator(Wrist wrist) {
        this.elevatorSpark1 = new SparkController(Constants.Setup.elevatorSpark1, new SparkControllerInfo().elevator());
        this.elevatorSpark2 = new SparkController(Constants.Setup.elevatorSpark2, new SparkControllerInfo().elevator());
       
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(Constants.Setup.elevatorSpark1);
        this.elevatorSpark2.spark.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        this.elevatorEncoder1 = elevatorSpark1.sparkEncode;
        this.elevatorEncoder2 = elevatorSpark2.sparkEncode;

        this.elevatorController1 = elevatorSpark1.sparkControl;
        this.elevatorController2 = elevatorSpark2.sparkControl;

        this.wrist = wrist;

        elevProfiledPID = new ProfiledPIDController(Constants.PID.elevatorPID[0], Constants.PID.elevatorPID[1], Constants.PID.elevatorPID[2],
          new TrapezoidProfile.Constraints(1.0, 1.0));    // TODO - find trapezoid constraits that work.
        elevProfiledPID.disableContinuousInput();              // Our sensor isn't continuous because it doesn't loop around. We expect max and min values.
        elevProfiledPID.reset(elevatorEncoder1.getPosition()); // TODO - figure out homing procedure?
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Motor 1 Velocity", elevatorEncoder1.getVelocity());
        SmartDashboard.putNumber("Elevator Motor 2 Velocity", elevatorEncoder2.getVelocity());
    }

    /**
     * Currently returns the height of the elevator, in motor rotations.
     * Once we get a gear ratio and distance per rotation, we can return inches of height.
     * 
     * @return the height of the elevator, in motor rotations
     */
    public double getHeight() {
        return elevatorEncoder1.getPosition();
    }

    public void setVoltage(double voltage) {
        double sdAngle = Constants.Elevator.selfDestructAngle;
        double sdToler = Constants.Elevator.selfDestructTolerance;

        if((sdAngle - sdToler < wrist.getAngle()) && (wrist.getAngle() < sdAngle + sdToler)) {
            return;
        }
        
        voltage = MathUtil.clamp(voltage, -Constants.Elevator.maxVoltage, Constants.Elevator.maxVoltage);

        if (getHeight() <= Constants.Elevator.softHeightMinimum) {
            voltage = MathUtil.clamp(voltage, -getHeight(), Constants.Elevator.maxVoltage);
        }
        elevatorController1.setReference(voltage, SparkBase.ControlType.kVoltage);
        //elevatorController2.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        elevatorController1.setReference(percent, SparkBase.ControlType.kDutyCycle);
        elevatorController2.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }

    public Command getSysIDRoutine() {
        /* TODO - do we have good config settings? We do not.
         * Don't run until after the subsystem is configured.
         */ 
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.5),
                Volts.of(1),
                Seconds.of(2)), 
            new SysIdRoutine.Mechanism(
                (voltage) -> setVoltage(getHeight()),
                null,
                this,
                "Elevator")
        );

        return new InstantCommand(
            () -> setVoltage(0)).andThen(
            new WaitCommand(0.25)).andThen(
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)).andThen(
            new WaitCommand(0.25)).andThen(
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)).andThen(
            new WaitCommand(0.25)).andThen(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)).andThen(
            new WaitCommand(0.25)).andThen(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    };
}