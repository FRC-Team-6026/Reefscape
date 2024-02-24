package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SVA.ShooterWheelsSVA[0],Constants.SVA.ShooterWheelsSVA[1],Constants.SVA.driveMotorsSVA[2]);


    public ShooterWheels(){
        this.leftFlyWheel = new SparkController(Constants.Setup.leftWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.left));
        this.rightFlyWheel = new SparkController(Constants.Setup.rightWheel, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.right));

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
