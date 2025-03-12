package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
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

    public SlewRateLimiter wristLimiter = new SlewRateLimiter(Constants.Wrist.maxAccel);
    public double currentVoltage;

    public double targetAngle;
    private static double targetMinAngle = Constants.Wrist.minimumAngle; // Minimum angle in degrees
    private static double targetMaxAngle = Constants.Wrist.maximumAngle; // Maximum angle in degrees

    public double lastVoltageAttempt;

    public Wrist() {
        this.wristSpark = new SparkController(Constants.Setup.wristSpark, new SparkControllerInfo().shooterWrist());
        
        this.wristController = wristSpark.sparkControl;
        
        wristEncoder = wristSpark.sparkEncode;
        wristAbsolute = wristSpark.sparkAbsoluteEncoder;
        
        wristPID = new ProfiledPIDController(Constants.PID.wristPID[0], Constants.PID.wristPID[1], Constants.PID.wristPID[2],
          new TrapezoidProfile.Constraints(Constants.Wrist.maxSpeed, Constants.Wrist.maxAccel));    // TODO - find trapezoid constraits that work. I think this is set to 15 deg/s
        wristPID.disableContinuousInput();
        wristPID.reset(wristAbsolute.getPosition());
        
        targetAngle = getAngle();
    }

    public void addAngle(double angle) {
        targetAngle += angle;
    }

    public void setAngle(double setToAngle) {
        // TODO - lets change this to a clamp with MathUtil.clamp
        if(setToAngle < targetMinAngle){
            setToAngle = targetMinAngle;
        } else if (setToAngle > targetMaxAngle){
            setToAngle = targetMaxAngle;
        }
        targetAngle = setToAngle;
    }
    
    /**
     * 
     * @return the angle, in degrees, of the angle the wrist should be moving to.
     */
    public double getTargetAngle() {
        return targetAngle;
    }
    
    /**
     * 
     * @return the angle, in degrees, of the wrist
     */
    public double getAngle() {
        return wristAbsolute.getPosition() * 360;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", wristAbsolute.getPosition());
        SmartDashboard.putNumber("Wrist Integrated Encoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist total Voltage", wristSpark.spark.getBusVoltage());
    }

    public void inputVoltage(double voltage) {
        voltage = wristLimiter.calculate(voltage);
        setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -Constants.Wrist.maxVoltage, Constants.Wrist.maxVoltage);
        
        currentVoltage = voltage;
        wristController.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent) {
        percent = percent/100;
        wristController.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}
