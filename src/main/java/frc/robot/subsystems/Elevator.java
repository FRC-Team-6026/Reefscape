package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    public SparkController elevatorSpark1;
    public SparkController elevatorSpark2;
    
    public RelativeEncoder elevatorEncoder1;
    public RelativeEncoder elevatorEncoder2;

    public SparkClosedLoopController elevatorController1;
    public SparkClosedLoopController elevatorController2;

    public ProfiledPIDController elevProfiledPID;

    public Elevator() {
        this.elevatorSpark1 = new SparkController(Constants.Setup.elevatorSpark1, new SparkControllerInfo().elevator());
        this.elevatorSpark2 = new SparkController(Constants.Setup.elevatorSpark2, new SparkControllerInfo().elevator());
       
        this.elevatorEncoder1 = elevatorSpark1.sparkEncode;
        this.elevatorEncoder2 = elevatorSpark2.sparkEncode;

        this.elevatorController1 = elevatorSpark1.sparkControl;
        this.elevatorController2 = elevatorSpark2.sparkControl;

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
        //return elevatorEncoder1.getPosition() * gearRatio * inchesPerRotation;
    }

    public void setVoltage(double voltage) {
        if(voltage < -Constants.Elevator.maxVoltage){
            voltage = -Constants.Elevator.maxVoltage;
        } else if (voltage > Constants.Elevator.maxVoltage){
            voltage = Constants.Elevator.maxVoltage;
        }
        elevatorController1.setReference(voltage, SparkBase.ControlType.kVoltage);
        elevatorController2.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        elevatorController1.setReference(percent, SparkBase.ControlType.kDutyCycle);
        elevatorController2.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}