package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {

    private SparkController leftFlyWheel;
    private SparkController rightFlyWheel;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private SparkClosedLoopController leftMotor;
    private SparkClosedLoopController rightMotor;


    public CoralIntake(){
        this.leftFlyWheel = new SparkController(Constants.Setup.ProtoMotor1, new SparkControllerInfo().prototype());
        this.rightFlyWheel = new SparkController(Constants.Setup.ProtoMotor2, new SparkControllerInfo().prototype());
       
        this.leftEncoder = leftFlyWheel.sparkEncode;
        this.rightEncoder = rightFlyWheel.sparkEncode;

        this.leftMotor = leftFlyWheel.sparkControl;
        this.rightMotor = rightFlyWheel.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Prototype Motor 1 Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Prototype Motor 2 Velocity", rightEncoder.getVelocity());
    }

    public void setVoltage(double voltage){
        if(voltage < -Constants.Prototype.maxVoltage){
            voltage = -Constants.Prototype.maxVoltage;
        } else if (voltage > Constants.Prototype.maxVoltage){
            voltage = Constants.Prototype.maxVoltage;
        }
        leftMotor.setReference(voltage, SparkBase.ControlType.kVoltage);
        rightMotor.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        leftMotor.setReference(percent, SparkBase.ControlType.kDutyCycle);
        rightMotor.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }

}