package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private SparkController elevatorSpark1;
    private SparkController elevatorSpark2;
    
    private RelativeEncoder elevatorEncoder1;
    private RelativeEncoder elevatorEncoder2;

    private SparkClosedLoopController elevatorController1;
    private SparkClosedLoopController elevatorController2;

    public Elevator() {
        this.elevatorSpark1 = new SparkController(Constants.Setup.elevatorSpark1, new SparkControllerInfo().elevator());
        this.elevatorSpark2 = new SparkController(Constants.Setup.elevatorSpark2, new SparkControllerInfo().elevator());
       
        this.elevatorEncoder1 = elevatorSpark1.sparkEncode;
        this.elevatorEncoder2 = elevatorSpark2.sparkEncode;

        this.elevatorController1 = elevatorSpark1.sparkControl;
        this.elevatorController2 = elevatorSpark2.sparkControl;
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