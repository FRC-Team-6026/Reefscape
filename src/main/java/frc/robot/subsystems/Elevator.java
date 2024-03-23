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

    // private ElevatorFeedforward feedForward = new ElevatorFeedforward(Constants.SVA.elevatorMotorSVA[0], Constants.SVA.elevatorMotorSVA[1], Constants.SVA.driveMotorsSVA[2]);

    private boolean deployed;
    
    public Elevator() {

        this.elevatorController = new SparkController(Constants.Setup.elevarorMotor, new SparkControllerInfo().elavator());
        
        this.elevatorEncoder = elevatorController.sparkEncode;

        this.elevatorPIDController = elevatorController.sparkControl;
        // elevatorEncoder.setPositionConversionFactor(0)       // Use if we change from rotations to distance units
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElevatorMotorVelocity", elevatorEncoder.getVelocity());
    }

    public void togglePosition() {
        deployed = !deployed;
        //if (deployed)
        //elevatorPIDController.setReference(Constants.Elevator.deployedPosition, CANSparkBase.ControlType.kPosition, 0, feedForward.calculate(Constants.Elevator.maxVel));
    }

    public void setVelocity(double tangentialVelocity) {
        if(tangentialVelocity < -Constants.Elevator.maxVel){
            tangentialVelocity = -Constants.Elevator.maxVel;
        } else if (tangentialVelocity > Constants.Elevator.maxVel){
            tangentialVelocity = Constants.Elevator.maxVel;
        }

        double elevatorLiftBalance = 1.5;

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