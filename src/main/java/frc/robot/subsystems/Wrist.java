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
    public double lastVelocity;

    public double lastVoltageAttempt;

    public Elevator s_Elevator;

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
        lastVelocity = 0;

        wristEncoder.setPosition(getAngle());
    }


    
    /** @return the angle, in degrees, that the wrist is actually at */
    public double getAngle() { return wristAbsolute.getPosition() * 360; }
    
    /** Sets the angle the wrist will track to. */
    public void setTargetAngle(double angle) { targetAngle = angle; }
    /** Changes the angle the wrist will track to by @param angle. */
    public void addTargetAngle(double angle) { targetAngle += angle; }
    
    /** @return the angle, in degrees, of the angle the wrist should be moving to.
     * The wrist will try to move to this angle on doNextVoltage() unless it's past the self-destruct angle.
     */
    public double getTargetAngle() { return targetAngle; }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", wristAbsolute.getPosition());
        SmartDashboard.putNumber("Wrist Integrated Encoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist total Voltage", wristSpark.spark.getBusVoltage());
    }

    /** This function calculates a voltage to send to the controller, based on where the wrist is and where it should be.
     * It then calls setVoltage to actually move the wrist.
     */
    public void doNextVoltage() {
        double acceptableTargetAngle = MathUtil.clamp(targetAngle, s_Elevator.getHeight() > 2 ? Constants.Elevator.selfDestructAngle + 5 : Constants.Wrist.minimumAngle, Constants.Wrist.maximumAngle);
        wristPID.setGoal(acceptableTargetAngle);

        double velocity = wristPID.getSetpoint().velocity;
        double ff = 
            Constants.SVA.WristSVA[0] * MathUtil.applyDeadband(Math.signum(acceptableTargetAngle - getAngle()), Constants.Wrist.angleTolerance) +
            Constants.SVA.WristSVA[1] * velocity +
            Constants.SVA.WristSVA[2] * (velocity - lastVelocity);
        
        double voltage = wristPID.calculate(getAngle()) + ff;
        if (voltage < Constants.Electrical.neoMinVoltage)   { setDutyCycle(0); }
        else                                                { setVoltage(voltage); }
    }

    /*
    public void setPositionAndVoltage(double angle) {
        targetAngle = angle;
        double acceptableTargetAngle = MathUtil.clamp(angle, s_Elevator.getHeight() > 2 ? Constants.Elevator.selfDestructAngle + 5 : Constants.Wrist.minimumAngle, Constants.Wrist.maximumAngle);
        
        double nextError = acceptableTargetAngle - getAngle();
        double voltage = 
            (nextError * Constants.PID.wristPID[0]) *
            (lastError * Constants.PID.wristPID[2])
            +
            (MathUtil.applyDeadband(Math.signum(acceptableTargetAngle - getAngle()),Constants.Wrist.angleTolerance) * Constants.SVA.WristSVA[0]);

        setVoltage(voltage);
    }
    */

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
