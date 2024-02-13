package frc.robot.subsystems;

import javax.swing.RowFilter.ComparisonType;
import javax.swing.text.html.HTMLDocument.HTMLReader.TagAction;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private SparkController topRoller;
    private SparkController bottomRoller;
    
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder; 

    private SparkPIDController topController;
    private SparkPIDController bottomController;

    private SimpleMotorFeedforward feedForward = 
        new SimpleMotorFeedforward(
          Constants.SVA.intakeRollersSVA[0], Constants.SVA.intakeRollersSVA[1], Constants.SVA.driveMotorsSVA[2]);

    

    public Intake(){

        this.topRoller = new SparkController(Constants.Setup.topRoller, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.left));
        this.bottomRoller = new SparkController(Constants.Setup.bottomRoller, new SparkControllerInfo().shooterWheel(Constants.Setup.shooterInverts.right));
        
        this.topEncoder = topRoller.sparkEncode;
        this.bottomEncoder = bottomRoller.sparkEncode;

        this.topController = topRoller.sparkControl;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("TopRollerVelocity", topEncoder.getVelocity());
        SmartDashboard.putNumber("BottomRollerVelocity", bottomEncoder.getVelocity());
    }

    public void setVelocity(double tangentialVelocity){
        tangentialVelocity = tangentialVelocity/Constants.Intake.intakeRollerReduction;
        if(tangentialVelocity < Constants.Intake.minTanVel){
            tangentialVelocity = Constants.Intake.minTanVel;
        } else if (tangentialVelocity > Constants.Intake.maxTanVel){
            tangentialVelocity = Constants.Intake.maxTanVel;
        }
        double tangentialToMotorOutput = tangentialVelocity * Constants.Intake.rollerMotortoOutputConversion;
        topController.setReference(tangentialToMotorOutput, CANSparkBase.ControlType.kVelocity, 0, feedForward.calculate(tangentialVelocity));
        bottomController.setReference(tangentialToMotorOutput, CANSparkBase.ControlType.kVelocity, 0, feedForward.calculate(tangentialVelocity));
    }

    public void setDutyCylce(double percent){
        percent = percent/100;
        topController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
        bottomController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }

    
}