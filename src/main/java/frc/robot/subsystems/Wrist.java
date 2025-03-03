package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

    private SparkController wristSpark;

    public RelativeEncoder wristEncoder;
    public AbsoluteEncoder wristAbsolute;

    public SparkClosedLoopController wristController;

    public ProfiledPIDController wristPID;

    private Timer WristTimer;

    public SlewRateLimiter wristLimiter = new SlewRateLimiter(Constants.Wrist.maxAccel);
    public double currentVoltage;

    private double targetAngle;
    private static double targetMinAngle = Constants.Wrist.minimumAngle; // Minimum angle in degrees
    private static double targetMaxAngle = Constants.Wrist.maximumAngle; // Maximum angle in degrees

    public boolean isTrackingAngle;
    public double lastVoltageAttempt;

    public Wrist() {

        WristTimer = new Timer();

        this.wristSpark = new SparkController(Constants.Setup.wristSpark, new SparkControllerInfo().shooterWrist());
        
        this.wristController = wristSpark.sparkControl;
        
        wristEncoder = wristSpark.sparkEncode;
        wristAbsolute = wristSpark.sparkAbsoluteEncoder;
        
        wristPID = new ProfiledPIDController(Constants.PID.wristPID[0], Constants.PID.wristPID[1], Constants.PID.wristPID[2],
          new TrapezoidProfile.Constraints(Constants.Wrist.maxSpeed, Constants.Wrist.maxAccel));    // TODO - find trapezoid constraits that work. I think this is set to 15 deg/s
        wristPID.disableContinuousInput();
        wristPID.reset(wristAbsolute.getPosition() * 360);
        
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

        wristController.setReference(targetAngle, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    
    public double getAngle() {
        return wristAbsolute.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", 0.0);
        SmartDashboard.putNumber("Wrist Angle", wristAbsolute.getPosition() * 360);
        SmartDashboard.putNumber("Wrist Integrated Encoder", 0.0);
        SmartDashboard.putNumber("Wrist Integrated Encoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist total Voltage", lastVoltageAttempt);
    }

    public void inputVoltage(double voltage) {
        voltage = wristLimiter.calculate(voltage);
        setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        if(voltage < -Constants.Wrist.maxVoltage){
            voltage = -Constants.Wrist.maxVoltage;
        } else if (voltage > Constants.Wrist.maxVoltage){
            voltage = Constants.Wrist.maxVoltage;
        }
        currentVoltage = voltage;
        wristController.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent) {
        percent = percent/100;
        wristController.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}
