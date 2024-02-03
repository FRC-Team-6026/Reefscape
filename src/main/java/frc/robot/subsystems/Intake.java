package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private SparkController topRoler;
    private SparkController bottomRoler;

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder; 


    

    public Intake(){

        this.topRoler = new SparkController(Constants.Setup.leftWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.left));
        this.bottomRoler = new SparkController(Constants.Setup.rightWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.right));

        this.topMotor = topRoler.spark;
        this.bottomMotor = bottomRoler.spark;
        
        this.topEncoder = topRoler.sparkEncode;
        this.bottomEncoder = bottomRoler.sparkEncode;
    }

    
}