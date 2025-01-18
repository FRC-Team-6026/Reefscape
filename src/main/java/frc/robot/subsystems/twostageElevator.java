package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class twostageElevator extends SubsystemBase {

    private SparkController protoMotor1;
    private SparkController protoMotor2;
    
    private RelativeEncoder protoEncoder1;
    private RelativeEncoder protoEncoder2;

    private SparkClosedLoopController protoController1;
    private SparkClosedLoopController protoController2;

    public twostageElevator() {
        this.protoMotor1 = new SparkController(Constants.Setup.ProtoMotor1, new SparkControllerInfo().prototype());
        this.protoMotor2 = new SparkController(Constants.Setup.ProtoMotor2, new SparkControllerInfo().prototype());
       
        this.protoEncoder1 = protoMotor1.sparkEncode;
        this.protoEncoder2 = protoMotor2.sparkEncode;

        this.protoController1 = protoMotor1.sparkControl;
        this.protoController2 = protoMotor2.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Prototype Motor 1 Velocity", protoEncoder1.getVelocity());
        SmartDashboard.putNumber("Prototype Motor 2 Velocity", protoEncoder2.getVelocity());
    }

    public void setVoltage(double voltage){
        if(voltage < -Constants.Prototype.maxVoltage){
            voltage = -Constants.Prototype.maxVoltage;
        } else if (voltage > Constants.Prototype.maxVoltage){
            voltage = Constants.Prototype.maxVoltage;
        }
        protoController1.setReference(voltage, SparkBase.ControlType.kVoltage);
        protoController2.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        protoController1.setReference(percent, SparkBase.ControlType.kDutyCycle);
        protoController2.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}