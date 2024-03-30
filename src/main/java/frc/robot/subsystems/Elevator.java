package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private SparkController elevatorController;
    
    private RelativeEncoder elevatorEncoder;

    private SparkPIDController elevatorPIDController;
    
    public Elevator() {

        this.elevatorController = new SparkController(Constants.Setup.elevatorMotor, new SparkControllerInfo().elevator());
        
        this.elevatorEncoder = elevatorController.sparkEncode;

        this.elevatorPIDController = elevatorController.sparkControl;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElevatorMotorEncoder", elevatorEncoder.getPosition());
    }

    public void setVoltage(double voltage) {
        if(voltage < -Constants.Elevator.maxVoltage){
            voltage = -Constants.Elevator.maxVoltage;
        } else if (voltage > Constants.Elevator.maxVoltage){
            voltage = Constants.Elevator.maxVoltage;
        }

        if(elevatorEncoder.getPosition() <= Constants.Elevator.stowedPosition){
            voltage = Math.max(0, voltage);
        } else if (elevatorEncoder.getPosition() >= Constants.Elevator.deployedPosition){
            voltage = Math.min(0, voltage);
        }

        elevatorPIDController.setReference(voltage, CANSparkBase.ControlType.kVoltage, 0);
    }

    public void setDutyCylce(double percent) {
        percent = percent/100;
        elevatorPIDController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

    
} 