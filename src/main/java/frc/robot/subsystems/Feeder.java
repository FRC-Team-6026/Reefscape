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

public class Feeder extends SubsystemBase {

    private SparkController feederController;

    private RelativeEncoder feederEncoder;

    private SparkPIDController feederPIDController; 

    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.SVA.ShooterWheelsSVA[0],Constants.SVA.ShooterWheelsSVA[1],Constants.SVA.driveMotorsSVA[2]);


    public Feeder(){
        this.feederController = new SparkController(Constants.Setup.feedRoller, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.left));
        
        this.feederEncoder = feederController.sparkEncode;
       
        this.feederPIDController = feederController.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("feederVelocity", feederEncoder.getVelocity());
    }

    public void setVelocity(double tangentialVelocity){
        if(tangentialVelocity < Constants.Feeder.minTanVel){
            tangentialVelocity = Constants.Feeder.minTanVel;
        } else if (tangentialVelocity > Constants.Feeder.maxTanVel){
            tangentialVelocity = Constants.Feeder.maxTanVel;
        }
        
        feederController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVelocity, 0, feedForward.calculate(tangentialVelocity/Constants.ConversionFactors.feederBaseConversionFactor));
    }

    public void setDutyCylce(double percent){
        percent = percent/100;
        feederPIDController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

}
