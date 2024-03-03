package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private SparkController elevatorController;
    
    private RelativeEncoder elevatorEncoder;

    private SparkPIDController elevatorPIDController;

    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.SVA.elevatorMotorSVA[0], Constants.SVA.elevatorMotorSVA[1], Constants.SVA.driveMotorsSVA[2]);

    public Elevator(){

        this.elevatorController = new SparkController(Constants.Setup.elevarorMotor, new SparkControllerInfo().elavator());
        
        this.elevatorEncoder = elevatorController.sparkEncode;

        this.elevatorPIDController = elevatorController.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ElevatorMotorVelocity", elevatorEncoder.getVelocity());
    }

    public void setVelocity(double tangentialVelocity){
        if(tangentialVelocity < Constants.Elevator.maxTanVel){
            tangentialVelocity = Constants.Elevator.minTanVel;
        } else if (tangentialVelocity > Constants.Elevator.maxTanVel){
            tangentialVelocity = Constants.Elevator.maxTanVel;
        }
        elevatorPIDController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVelocity, 0, feedForward.calculate(tangentialVelocity/Constants.ConversionFactors.elevatorBaseConversionFactor));
    }

    public void setDutyCylce(double percent){
        percent = percent/100;
        elevatorPIDController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

    
} 