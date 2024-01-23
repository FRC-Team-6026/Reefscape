package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class ShooterWheels extends SubsystemBase {

    private SparkController leftFlyWheel;
    private SparkController rightFlyWheel;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder; 



    public ShooterWheels(){
        this.leftFlyWheel = new SparkController(Constants.Setup.leftWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.left));
        this.rightFlyWheel = new SparkController(Constants.Setup.rightWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.right));

        this.leftMotor = leftFlyWheel.spark;
        this.rightMotor = rightFlyWheel.spark;
        
        this.leftEncoder = leftFlyWheel.sparkEncode;
        this.rightEncoder = rightFlyWheel.sparkEncode;
    }

    
}
