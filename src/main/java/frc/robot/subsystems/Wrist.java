package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

    public SparkClosedLoopController WristPidController;

    public SparkController WristMotor;

    public DutyCycleEncoder WristEncoder;
    // public RelativeEncoder WristEncoder;

    //private PIDController wristPID;
    public ProfiledPIDController wristPID;

    private Timer WristTimer;

    private double targetAngle;
    private static double targetMinAngle = Constants.Wrist.minimumAngle; // Minimum angle in degrees
    private static double targetMaxAngle = Constants.Wrist.maximumAngle; // Maximum angle in degrees

    public boolean isTrackingAngle;
    public double lastVoltageAttempt;

    public Wrist() {

        WristTimer = new Timer();

        int channel = 3;
        lastVoltageAttempt = 0;
        WristEncoder = new DutyCycleEncoder(channel);
                // Base units are full motor rotations
        // WristEncoder.setPositionConversionFactor(360 / Constants.Wrist.gearReduction);    // 360 deg/subsystem_rotation * 11/24 subsystem_rotations/motor_rotation
                // Base units are RPM (full motor rotations per minuite)
        // WristEncoder.setVelocityConversionFactor(360 / (Constants.Wrist.gearReduction * 60)); // 360 deg/subsystem_rotation * 11/24 subsystem_rotations/motor_rotation * 1/60 minutes/second
        
        this.WristMotor = new SparkController(Constants.Setup.wristMotor, new SparkControllerInfo().shooterWrist());
        this.WristPidController = WristMotor.sparkControl;

        // wristPID = new PIDController(Constants.PID.wristPID[0], Constants.PID.wristPID[1], Constants.PID.wristPID[2]);
        wristPID = new ProfiledPIDController(Constants.PID.wristPID[0], Constants.PID.wristPID[1], Constants.PID.wristPID[2],
          new TrapezoidProfile.Constraints(Constants.Wrist.maxTurnSpeed, Constants.Wrist.maxAccel));    // TODO - find trapezoid constraits that work. I think this is set to 15 deg/s
        wristPID.disableContinuousInput();
        wristPID.reset(WristEncoder.get() * 360);

        isTrackingAngle = false;
    }

    public void addAngle(double changeAngle) {
        setAngle(targetAngle + (Constants.Wrist.maxSpeed * changeAngle * Math.min(WristTimer.get(), 1)));
        WristTimer.restart();
    }

    public void setAngle(double setToAngle) {
        if(setToAngle < targetMinAngle){
            setToAngle = targetMinAngle;
        } else if (setToAngle > targetMaxAngle){
            setToAngle = targetMaxAngle;
        }
        targetAngle = setToAngle;

        WristPidController.setReference(targetAngle, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", WristEncoder.get() * 360);
        SmartDashboard.putNumber("Wrist total Voltage", lastVoltageAttempt);
    }

    // TODO - Insert a function for the joystick to move up and down smoothly

    public void setDutyCycle(double percent) {
        percent = percent/100;
        WristPidController.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}
