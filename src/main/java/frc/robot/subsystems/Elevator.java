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

    private boolean deployed;
    
    public Elevator() {

        this.elevatorController = new SparkController(Constants.Setup.elevarorMotor, new SparkControllerInfo().elavator());
        
        this.elevatorEncoder = elevatorController.sparkEncode;

        this.elevatorPIDController = elevatorController.sparkControl;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElevatorMotorEncoder", elevatorEncoder.getPosition());
    }

    public void togglePosition() {
        deployed = !deployed;
     }

    public void setVelocity(double tangentialVelocity) {
        if(tangentialVelocity < -Constants.Elevator.maxVel){
            tangentialVelocity = -Constants.Elevator.maxVel;
        } else if (tangentialVelocity > Constants.Elevator.maxVel){
            tangentialVelocity = Constants.Elevator.maxVel;
        }

        /*    // TODO - enable after we record limits
        if(elevatorEncoder.getPosition() <= Constants.Elevator.stowedPosition){
            tangentialVelocity = Math.max(0, tangentialVelocity);
        } else if (elevatorEncoder.getPosition() > Constants.Elevator.deployedPosition){
            tangentialVelocity = Math.min(0, tangentialVelocity);
        }
        */

        double elevatorLiftBalance = 1.6;

        if(tangentialVelocity > 0) {
            tangentialVelocity *= elevatorLiftBalance;
        }

        elevatorPIDController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVoltage, 0);
    }

    public void setDutyCylce(double percent) {
        percent = percent/100;
        elevatorPIDController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

    
} 