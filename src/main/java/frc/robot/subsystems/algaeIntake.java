package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {

    private SparkController algaeSpark1;
    private SparkController algaeSpark2;
    
    private RelativeEncoder algaeEncoder1;
    private RelativeEncoder algaeEncoder2;

    private SparkClosedLoopController algaeController1;
    private SparkClosedLoopController algaeController2;

    public AlgaeIntake() {
        this.algaeSpark1 = new SparkController(Constants.Setup.algaeSpark1, new SparkControllerInfo().algaeIntake());
        this.algaeSpark2 = new SparkController(Constants.Setup.algaeSpark2, new SparkControllerInfo().algaeIntake());
       
        this.algaeEncoder1 = algaeSpark1.sparkEncode;
        this.algaeEncoder2 = algaeSpark2.sparkEncode;

        this.algaeController1 = algaeSpark1.sparkControl;
        this.algaeController2 = algaeSpark2.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Algae Motor 1 Velocity", algaeEncoder1.getVelocity());
        SmartDashboard.putNumber("Algae Motor 2 Velocity", algaeEncoder2.getVelocity());
    }

    public void setVoltage(double voltage){
        if(voltage < -Constants.AlgaeIntake.maxVoltage){
            voltage = -Constants.AlgaeIntake.maxVoltage;
        } else if (voltage > Constants.AlgaeIntake.maxVoltage){
            voltage = Constants.AlgaeIntake.maxVoltage;
        }
        algaeController1.setReference(voltage, SparkBase.ControlType.kVoltage);
        algaeController2.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        algaeController1.setReference(percent, SparkBase.ControlType.kDutyCycle);
        algaeController2.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}