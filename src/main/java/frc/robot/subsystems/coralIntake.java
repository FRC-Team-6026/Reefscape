package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class coralIntake extends SubsystemBase {

    private SparkController leftFlyWheel;
    private SparkController rightFlyWheel;

    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder; 


    public coralIntake(){
       
        this.leftMotor = leftFlyWheel.spark;
        this.rightMotor = rightFlyWheel.spark;
        
        this.leftEncoder = leftFlyWheel.sparkEncode;
        this.rightEncoder = rightFlyWheel.sparkEncode;
    }

    //Initial set up for the periodic void.
    //Reminder to set up the values in the constants file

    @Override
    public void periodic(){
        SmartDashboard.putNumber(getName(), 0);
        SmartDashboard.putNumber(getName(), 0);
    }

}