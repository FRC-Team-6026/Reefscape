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

    private SparkPIDController topController;

    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.SVA.intakeRollersSVA[0], Constants.SVA.intakeRollersSVA[1], Constants.SVA.driveMotorsSVA[2]);

    public Elevator(){

        this.elevatorController = new SparkController(Constants.Setup.topRoller, new SparkControllerInfo().intake());
        
        this.elevatorEncoder = elevatorController.sparkEncode;

        this.topController = elevatorController.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("TopRollerVelocity", elevatorEncoder.getVelocity());
    }

    public void setVelocity(double tangentialVelocity){
        if(tangentialVelocity < Constants.Intake.minTanVel){
            tangentialVelocity = Constants.Intake.minTanVel;
        } else if (tangentialVelocity > Constants.Intake.maxTanVel){
            tangentialVelocity = Constants.Intake.maxTanVel;
        }
        topController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVelocity, 0, feedForward.calculate(tangentialVelocity/Constants.ConversionFactors.intakeBaseConversionFactor));
    }

    public void setDutyCylce(double percent){
        percent = percent/100;
        topController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

    
} 