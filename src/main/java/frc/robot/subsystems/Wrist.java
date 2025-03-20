package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

    private SparkController wristSpark;

    public RelativeEncoder wristEncoder;
    public AbsoluteEncoder wristAbsolute;

    public SparkClosedLoopController wristController;

    public ProfiledPIDController wristPID;

    public double currentVoltage;

    public double targetAngle;
    public double lastVelocity;

    public double lastVoltageAttempt;

    public Elevator s_Elevator;

    public Wrist() {
        this.wristSpark = new SparkController(Constants.Setup.wristSpark, new SparkControllerInfo().shooterWrist(),
            // -Constants.Wrist.maxVoltage, Constants.Wrist.maxVoltage,
            -0.25, 0.25,
            Constants.Wrist.maximumAngle, Constants.Wrist.minimumAngle);
            // null, null);
        
        this.wristController = wristSpark.sparkControl;
        
        wristEncoder = wristSpark.sparkEncode;
        wristAbsolute = wristSpark.sparkAbsoluteEncoder;
        
        wristEncoder.setPosition(wristAbsolute.getPosition() * 360.0);

        // wristPID = new ProfiledPIDController(Constants.PID.wristPID[0], Constants.PID.wristPID[1], Constants.PID.wristPID[2],
        //   new TrapezoidProfile.Constraints(Constants.Wrist.maxSpeed, Constants.Wrist.maxAccel));    // TODO - find trapezoid constraits that work. I think this is in deg/s
        // wristPID.disableContinuousInput();
        // wristPID.reset(wristAbsolute.getPosition());
        
        targetAngle = getAngle();
        lastVelocity = 0;

        //wristEncoder.setPosition(getAngle());
    }


    
    /** @return the angle, in degrees, that the wrist is actually at */
    public double getAngle() { return wristAbsolute.getPosition() * 360; }
    
    /** Sets the angle the wrist will track to. */
    @Deprecated
    public void setTargetAngle(double angle) { targetAngle = angle; }
    /** Changes the angle the wrist will track to by @param angle. */
    @Deprecated
    public void addTargetAngle(double angle) { targetAngle += angle; }
    
    /** @return the angle, in degrees, of the angle the wrist should be moving to.
     * The wrist will try to move to this angle on doNextVoltage() unless it's past the self-destruct angle.
     */
    public double getTargetAngle() { return targetAngle; }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", wristAbsolute.getPosition()*360);
        SmartDashboard.putNumber("Wrist Integrated Encoder", wristEncoder.getPosition());
    }

    /** This function calculates a voltage to send to the controller, based on where the wrist is and where it should be.
     * It then calls setVoltage to actually move the wrist.
     */
    @Deprecated
    public void doNextVoltage() {
        double acceptableTargetAngle = MathUtil.clamp(targetAngle, s_Elevator.getHeight() > 2 ? Constants.Elevator.selfDestructAngle + 5 : Constants.Wrist.minimumAngle, Constants.Wrist.maximumAngle);
        wristPID.setGoal(acceptableTargetAngle);

        double feedback = wristPID.calculate(getAngle());

        double velocity = wristPID.getSetpoint().velocity;
        double ff = 
            Constants.SVA.WristSVA[0] * Math.signum(MathUtil.applyDeadband(acceptableTargetAngle - getAngle(), Constants.Wrist.angleTolerance)) +
            //Constants.SVA.WristSVA[1] * velocity +
            Constants.SVA.WristSVA[1] * velocity;
            //Constants.SVA.WristSVA[2] * (velocity - lastVelocity);
        
        double voltage = feedback + ff;
        // lastVoltageAttempt = voltage;
        if (Math.abs(voltage) < Constants.Electrical.neoMinVoltage)   { setDutyCycle(0); }
        else                                                { setVoltage(voltage); }
    }

    public void setAngle(double angle) {
        angle = MathUtil.clamp(angle, Constants.Wrist.minimumAngle, Constants.Wrist.maximumAngle);
        wristController.setReference(angle, SparkBase.ControlType.kPosition);
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -Constants.Wrist.maxVoltage, Constants.Wrist.maxVoltage);
        
        currentVoltage = voltage;
        wristController.setReference(voltage, SparkBase.ControlType.kVoltage);
    }

    public void setDutyCycle(double percent) {
        wristPID.reset(getAngle());
        targetAngle = getAngle();
        percent = percent/100;
        wristController.setReference(percent, SparkBase.ControlType.kDutyCycle);
    }
}
