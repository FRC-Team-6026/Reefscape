package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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
    }

    public void setVoltage(double targetVoltage){
        if(targetVoltage < -Constants.Feeder.maxVoltage){
            targetVoltage = -Constants.Feeder.maxVoltage;
        } else if (targetVoltage > Constants.Feeder.maxVoltage){
            targetVoltage = Constants.Feeder.maxVoltage;
        }
        
        feederPIDController.setReference(targetVoltage, CANSparkBase.ControlType.kVoltage,0);
    }

    public void setDutyCylce(double percent){
        percent = percent/100;
        feederPIDController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

}
