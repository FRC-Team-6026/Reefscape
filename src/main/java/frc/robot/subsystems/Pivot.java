package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
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

    private PIDController pivotPID;

    private double targetAngle;
    private double targetMinAngle = 240; // Minimum angle in degrees
    private double targetMaxAngle = 310; // Maximum angle in degrees

    public Pivot() {
        int channel = 3; // Replace with actual channel

        PivotEncoder = new DutyCycleEncoder(channel);
        PivotEncoder.setDistancePerRotation(360.0); // Set the encode to use degrees
        PivotEncoder.setPositionOffset(0);
        
        this.PivotMotor = new SparkController(Constants.Setup.pivotMotor, new SparkControllerInfo().shooterPivot());
        this.PivotPidController = PivotMotor.sparkControl;

        pivotPID = new PIDController(Constants.PID.shooterPivotPID[0], Constants.PID.shooterPivotPID[1], Constants.PID.shooterPivotPID[2]);

        targetAngle = PivotEncoder.get();
    }

    public void setAngle(double targetAngle) {
        if(targetAngle < targetMinAngle){
            targetAngle = targetMinAngle;
        } else if (targetAngle > targetMaxAngle){
            targetAngle = targetMaxAngle;
        }
        this.targetAngle = targetAngle;

        // PivotPidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, 0);
    }

    public void periodic() {
        SmartDashboard.putNumber("PivotAngle", PivotEncoder.get());
        SmartDashboard.putNumber("PivotAbsolutePosition", PivotEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("PivotDistance", PivotEncoder.getDistance());
        PivotMotor.spark.setVoltage(pivotPID.calculate(getConvertedAngle(), targetAngle));
    }

    public double getConvertedAngle() {
        double angle = PivotEncoder.getDistance();
        if (angle < 0) {
            angle += 360;
        } else if (angle > 360) {
            angle -= 360;
        }
        return angle;
    }

    // TODO - Insert a function for the joystick to move up and down smoothly
    /*
    public void useShooterPivot(double targetAngle) {
        double encoderValue = PivotEncoder.get();
        double currentAngle = (encoderValue - 0.1) * (targetMaxAngle - targetMinAngle) / 0.8 + targetMinAngle;
        SmartDashboard.putNumber("Current Angle", currentAngle);
    }
    */

    public void setDutyCycle(double percent){
        percent = percent/100;
        PivotPidController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }
}
