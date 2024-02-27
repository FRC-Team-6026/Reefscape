package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    private double targetMinAngle = -45.0; // Minimum angle in degrees

    private double targetMaxAngle = 45.0; // Maximum angle in degrees

    private SimpleMotorFeedforward feedForward =  
    new SimpleMotorFeedforward(Constants.SVA.ShooterPivotSVA[0], Constants.SVA.ShooterPivotSVA[1], Constants.SVA.driveMotorsSVA[2]);

    public Pivot() {
        int channel = 2; // Replace with actual channel

        PivotEncoder = new DutyCycleEncoder(channel);

        PivotEncoder.setDistancePerRotation(360.0); // Assuming one full rotation = 360 degrees

        PivotEncoder.setDutyCycleRange(0.1, 0.9); // Set the encoder duty cycle range

        this.PivotMotor = new SparkController(Constants.Setup.pivotMotor, new SparkControllerInfo().shooterPivot());
        
        this.PivotPidController = PivotMotor.sparkControl;
    }

    public void setVelocity(double tangentialVelocity){
        if(tangentialVelocity < Constants.Intake.minTanVel){
            tangentialVelocity = Constants.Intake.minTanVel;
        } else if (tangentialVelocity > Constants.Intake.maxTanVel){
            tangentialVelocity = Constants.Intake.maxTanVel;
        }
        PivotPidController.setReference(tangentialVelocity, CANSparkBase.ControlType.kVelocity, 0, feedForward.calculate(tangentialVelocity/Constants.ConversionFactors.pivotBaseConversionFactor));
    }

    // TODO: Insert a function for the joystick to move up and down smoothly
    
    public void useShooterPivot(double targetAngle) {
        double encoderValue = PivotEncoder.get();
        double currentAngle = (encoderValue - 0.1) * (targetMaxAngle - targetMinAngle) / 0.8 + targetMinAngle;

        if (currentAngle >= targetMinAngle && currentAngle <= targetMaxAngle) {
            // Perform motor control based on target angle
           // shooterPivotController.set(targetAngle);
        } else {
            // Stop the motor or take corrective action
        }

        SmartDashboard.putNumber("Current Angle", currentAngle);
    }

    public void setDutyCycle(double percent){
        percent = percent/100;
        PivotPidController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }
}
