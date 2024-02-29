package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    private SparkPIDController PivotPidController;

    private SparkController PivotMotor;

    private DutyCycleEncoder PivotEncoder;

    private double targetMinAngle = -0.2; // Minimum angle in degrees

    private double targetMaxAngle = 0.1; // Maximum angle in degrees

    public Pivot() {
        int channel = 0; // Replace with actual channel

        PivotEncoder = new DutyCycleEncoder(channel);

        PivotEncoder.setDistancePerRotation(10.0); // Assuming one full rotation  10 degrees

        PivotEncoder.setDutyCycleRange(0.1, 0.9); // Set the encoder duty cycle range

        this.PivotMotor = new SparkController(Constants.Setup.pivotMotor, new SparkControllerInfo().shooterPivot());
        
        this.PivotPidController = PivotMotor.sparkControl;
    }

    public void setAngle(double targetAngle){
        if(targetAngle < targetMinAngle){
            targetAngle = targetMinAngle;
        } else if (targetAngle > targetMaxAngle){
            targetAngle = targetMaxAngle;
        }
        PivotPidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, 0);
    }

    // TODO - Insert a function for the joystick to move up and down smoothly
    
    public void useShooterPivot(double targetAngle) {
        double encoderValue = PivotEncoder.get();
        double currentAngle = (encoderValue - 0.1) * (targetMaxAngle - targetMinAngle) / 0.8 + targetMinAngle;
        SmartDashboard.putNumber("Current Angle", currentAngle);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        PivotPidController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }
}
