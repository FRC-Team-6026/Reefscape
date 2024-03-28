package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    public SparkPIDController PivotPidController;

    public SparkController PivotMotor;

    public DutyCycleEncoder PivotEncoder;
    //public RelativeEncoder PivotEncoder;

    //private PIDController pivotPID;
    public ProfiledPIDController pivotPID;
/*
    private Timer PivotTimer;

    private double targetAngle;
    private static double targetMinAngle = Constants.Pivot.minimumAngle; // Minimum angle in degrees
    private static double targetMaxAngle = Constants.Pivot.maximumAngle; // Maximum angle in degrees
 */
    public boolean isTrackingAngle;
    public double lastVoltageAttempt;

    public Pivot() {

        // PivotTimer = new Timer();

        int channel = 3;
        lastVoltageAttempt = 0;
        PivotEncoder = new DutyCycleEncoder(channel);
        //PivotEncoder.setDistancePerRotation(360.0); // Set the encoder to use degrees (we use absoluteposition, so it doesnt use this value)
        PivotEncoder.setPositionOffset(0);
        /*
        PivotEncoder = PivotMotor.sparkEncode;
                // Base units are full motor rotations
        PivotEncoder.setPositionConversionFactor(360 / Constants.Pivot.gearReduction);    // 360 deg/subsystem_rotation * 11/24 subsystem_rotations/motor_rotation
                // Base units are RPM (full motor rotations per minuite)
        PivotEncoder.setVelocityConversionFactor(360 / (Constants.Pivot.gearReduction * 60)); // 360 deg/subsystem_rotation * 11/24 subsystem_rotations/motor_rotation * 1/60 minutes/second

        PivotEncoder.setPosition(0);
         */
        
        this.PivotMotor = new SparkController(Constants.Setup.pivotMotor, new SparkControllerInfo().shooterPivot());
        this.PivotPidController = PivotMotor.sparkControl;

        // pivotPID = new PIDController(Constants.PID.pivotPID[0], Constants.PID.pivotPID[1], Constants.PID.pivotPID[2]);
        pivotPID = new ProfiledPIDController(Constants.PID.pivotPID[0], Constants.PID.pivotPID[1], Constants.PID.pivotPID[2],
          new TrapezoidProfile.Constraints(Constants.Pivot.maxTurnSpeed, Constants.Pivot.maxAccel));    // TODO - find trapezoid constraits that work. I think this is set to 15 deg/s
        pivotPID.disableContinuousInput();
        pivotPID.reset(PivotEncoder.getAbsolutePosition() * 360);

        isTrackingAngle = false;
    }
/*
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
 */
    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle", PivotEncoder.getAbsolutePosition() * 360);
        SmartDashboard.putBoolean("Is Pivot moving to exact angle?", isTrackingAngle);
        SmartDashboard.putNumber("Last Pivot Voltage Attempt", lastVoltageAttempt);
    }

    // TODO - Insert a function for the joystick to move up and down smoothly

    public void setDutyCycle(double percent) {
        percent = percent/100;
        PivotPidController.setReference(percent, CANSparkBase.ControlType.kDutyCycle);
    }
}
