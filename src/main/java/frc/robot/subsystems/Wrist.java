package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;

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

    public double currentVoltage;
    public double currentAccel;

    private double targetAngle;
    private static double targetMinAngle = Constants.Wrist.minimumAngle; // Minimum angle in degrees
    private static double targetMaxAngle = Constants.Wrist.maximumAngle; // Maximum angle in degrees

    public boolean isTrackingAngle;
    public double lastVoltageAttempt;

    public Wrist() {

        WristTimer = new Timer();

                // Base units are full motor rotations
        // wristAbsolute.setPositionConversionFactor(360 / Constants.Wrist.gearReduction);    // 360 deg/subsystem_rotation * 11/24 subsystem_rotations/motor_rotation
                // Base units are RPM (full motor rotations per minuite)
        // wristAbsolute.setVelocityConversionFactor(360 / (Constants.Wrist.gearReduction * 60)); // 360 deg/subsystem_rotation * 11/24 subsystem_rotations/motor_rotation * 1/60 minutes/second
        
        this.wristSpark = new SparkController(Constants.Setup.wristSpark, new SparkControllerInfo().shooterWrist());
        
        this.wristController = wristSpark.sparkControl;
        
        wristEncoder = wristSpark.sparkEncode;
        wristAbsolute = wristSpark.sparkAbsoluteEncoder;

        // wristPID = new PIDController(Constants.PID.wristPID[0], Constants.PID.wristPID[1], Constants.PID.wristPID[2]);
        
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
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Angle", 0.0);
        SmartDashboard.putNumber("Wrist Angle", wristAbsolute.getPosition() * 360);
        SmartDashboard.putNumber("Wrist Integrated Encoder", 0.0);
        SmartDashboard.putNumber("Wrist Integrated Encoder", wristEncoder.getPosition());
        //SmartDashboard.putNumber("Wrist total Voltage", lastVoltageAttempt);
    }

    // TODO - Insert a function for the joystick to move up and down smoothly
    public void inputVoltage(double voltage) {
        if(voltage < -Constants.Wrist.maxVoltage){
            voltage = -Constants.Wrist.maxVoltage;
        } else if (voltage > Constants.Wrist.maxVoltage){
            voltage = Constants.Wrist.maxVoltage;
        }

        double change = voltage * Constants.Wrist.voltageAccelFactor;
        currentAccel = currentAccel + change;

        if(currentAccel < -Constants.Wrist.maxAccel){
            currentAccel = -Constants.Wrist.maxAccel;
        } else if (currentAccel > Constants.Wrist.maxAccel){
            currentAccel = Constants.Wrist.maxAccel;
        }

        currentVoltage = currentVoltage + currentAccel;

        if(currentVoltage < -Constants.Wrist.maxSpeed){
            currentVoltage = -Constants.Wrist.maxSpeed;
        } else if (currentVoltage > Constants.Wrist.maxSpeed){
            currentVoltage = Constants.Wrist.maxSpeed;
        }

        setVoltage(currentVoltage);
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
