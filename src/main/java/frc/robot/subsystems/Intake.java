package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private SparkController topRoller;
    private SparkController bottomRoller;
    
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder; 

    private SparkPIDController topController;
    private SparkPIDController bottomController;

    private SimpleMotorFeedforward feedForward = 
        new SimpleMotorFeedforward(
          Constants.SVA.intakeRollersSVA[0], Constants.SVA.intakeRollersSVA[1], Constants.SVA.driveMotorsSVA[2]);

    

    public Intake(){

        this.topRoller = new SparkController(Constants.Setup.topRoller, new SparkControllerInfo().intake());
        this.bottomRoller = new SparkController(Constants.Setup.bottomRoller, new SparkControllerInfo().intake());
        
        this.topEncoder = topRoller.sparkEncode;
        this.bottomEncoder = bottomRoller.sparkEncode;

        this.topController = topRoller.sparkControl;
        this.bottomController = bottomRoller.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("TopRollerVelocity", topEncoder.getVelocity());
        SmartDashboard.putNumber("BottomRollerVelocity", bottomEncoder.getVelocity());
    }

    public void setVelocity(double tangentialVelocity){
        if(tangentialVelocity < Constants.Intake.minTanVel){
            tangentialVelocity = Constants.Intake.minTanVel;
        } else if (tangentialVelocity > Constants.Intake.maxTanVel){
            tangentialVelocity = Constants.Intake.maxTanVel;
        }
        topController.setReference(0.1 * (tangentialVelocity / Constants.Intake.rollerMotortoOutputConversion), CANSparkBase.ControlType.kVoltage);
        bottomController.setReference(0.1 * (tangentialVelocity / Constants.Intake.rollerMotortoOutputConversion), CANSparkBase.ControlType.kVoltage);
    }

    public void setDutyCylce(double percent){
        percent = percent/100;
        topController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
        bottomController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

    
}