package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

    private SparkController feederController;

    private RelativeEncoder feederEncoder;

    private SparkPIDController feederPIDController; 

    public Feeder(){
        this.feederController = new SparkController(Constants.Setup.feedRoller, new SparkControllerInfo().feeder());
        
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
        
        feederPIDController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVoltage, 0, Constants.Electical.feederHarcodedVoltage);
    }

    public void setDutyCylce(double percent){
        percent = percent/100;
        feederPIDController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

}
