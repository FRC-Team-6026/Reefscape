package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class ShooterWheels extends SubsystemBase {

    private SparkController leftFlyWheel;
    private SparkController rightFlyWheel;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder; 

    private SparkPIDController leftController;
    private SparkPIDController rightController;

    public ShooterWheels(){
        this.leftFlyWheel = new SparkController(Constants.Setup.leftWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.left));
        this.rightFlyWheel = new SparkController(Constants.Setup.rightWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.right));
        
        this.leftEncoder = leftFlyWheel.sparkEncode;
        this.rightEncoder = rightFlyWheel.sparkEncode;

        this.leftController = leftFlyWheel.sparkControl;
        this.rightController = rightFlyWheel.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("LeftFlywheelVelocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("RightFlywheelVelocity", rightEncoder.getVelocity());
    }

    public void setVelocity(double tangentialVelocity){
        if(tangentialVelocity < Constants.Shooter.minTanVel){
            tangentialVelocity = Constants.Shooter.minTanVel;
        } else if (tangentialVelocity > Constants.Shooter.maxTanVel){
            tangentialVelocity = Constants.Shooter.maxTanVel;
        }
        
        leftController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVoltage, 0, Constants.Electical.shooterHardcodedVoltage);
        rightController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVoltage, 0, Constants.Electical.shooterHardcodedVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        leftController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
        rightController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

}
