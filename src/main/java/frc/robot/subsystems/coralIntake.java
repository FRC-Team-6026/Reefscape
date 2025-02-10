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

    private SparkController coralSpark1;
    private SparkController coralSpark2;

    private RelativeEncoder coralEncoder1;
    private RelativeEncoder coralEncoder2;

    private SparkClosedLoopController coralController1;
    private SparkClosedLoopController coralController2;


    public CoralIntake(){
        this.coralSpark1 = new SparkController(Constants.Setup.coralSpark1, new SparkControllerInfo().coralIntake());
        this.coralSpark2 = new SparkController(Constants.Setup.coralSpark2, new SparkControllerInfo().coralIntake());
       
        this.coralEncoder1 = coralSpark1.sparkEncode;
        this.coralEncoder2 = coralSpark2.sparkEncode;

        this.coralController1 = coralSpark1.sparkControl;
        this.coralController2 = coralSpark2.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Coral Motor 1 Velocity", coralEncoder1.getVelocity());
        SmartDashboard.putNumber("Coral Motor 2 Velocity", coralEncoder2.getVelocity());
    }

    public void setVoltage(double voltage){
        if(voltage < -Constants.CoralIntake.maxVoltage){
            voltage = -Constants.CoralIntake.maxVoltage;
        } else if (voltage > Constants.CoralIntake.maxVoltage){
            voltage = Constants.CoralIntake.maxVoltage;
        }
        coralController1.setReference(voltage, SparkBase.ControlType.kVoltage);
        coralController2.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        coralController1.setReference(percent, SparkBase.ControlType.kDutyCycle);
        coralController2.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }

}