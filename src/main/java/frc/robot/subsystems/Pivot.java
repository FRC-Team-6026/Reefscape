package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
    //private ProfiledPIDController pivotPID;   // TODO - switch to trapezoid profiling  (M4 push)

    private Timer PivotTimer;

    private double targetAngle;
    private static double targetMinAngle = Constants.Pivot.minimumAngle; // Minimum angle in degrees
    private static double targetMaxAngle = Constants.Pivot.maximumAngle; // Maximum angle in degrees

    public Pivot() {
        int channel = 3; // Replace with actual channel

        PivotTimer = new Timer();

        PivotEncoder = new DutyCycleEncoder(channel);
        PivotEncoder.setDistancePerRotation(360.0); // Set the encode to use degrees
        PivotEncoder.setPositionOffset(0);
        
        this.PivotMotor = new SparkController(Constants.Setup.pivotMotor, new SparkControllerInfo().shooterPivot());
        this.PivotPidController = PivotMotor.sparkControl;

        pivotPID = new PIDController(Constants.PID.pivotPID[0], Constants.PID.pivotPID[1], Constants.PID.pivotPID[2]);
        // pivotPID = new ProfiledPIDController(Constants.PID.Pivot[0], Constants.PID.Pivot[1], Constants.PID.Pivot[2], new TrapezoiProfile.Constraints(5, 10));    // TODO - find trapezoid constraits that work. 5 deg/s is probably way too slow   (M4 push)

        targetAngle = PivotEncoder.getAbsolutePosition() * 360;
    }

    public void addAngle(double changeAngle) {
        setAngle(targetAngle + (Constants.Pivot.maxSpeed * changeAngle * Math.min(PivotTimer.get(), 1)));
        PivotTimer.restart();
    }

    public void setAngle(double setToAngle) {
        if(setToAngle < targetMinAngle){
            setToAngle = targetMinAngle;
        } else if (setToAngle > targetMaxAngle){
            setToAngle = targetMaxAngle;
        }
        targetAngle = setToAngle;

        // PivotPidController.setReference(targetAngle, CANSparkBase.ControlType.kPosition, 0);
    }

    public void periodic() {
        SmartDashboard.putNumber("PivotAngle", PivotEncoder.get());
        SmartDashboard.putNumber("PivotAbsolutePosition", PivotEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("PivotDistance", PivotEncoder.getDistance());
        PivotMotor.spark.setVoltage(pivotPID.calculate(getConvertedAngle(), targetAngle)); // TODO - add feedforward afterall?  (M4 push)
    }

    public double getConvertedAngle() { // TODO - possibly rework to use absolute, then multiply for distance  (M4 push)
        double angle = PivotEncoder.getAbsolutePosition() * 360;
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
