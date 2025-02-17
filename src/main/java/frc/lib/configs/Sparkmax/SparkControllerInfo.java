package frc.lib.configs.Sparkmax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.*;

public class SparkControllerInfo {
    public Usage canbusUse;
    public int currentLim;
    public boolean invert;
    public boolean alternateAbsolute;    // is a throughbore plugged in as an absolute encoder?
    public IdleMode idleMode;
    public double posConversion;
    public double velConversion;
    public double[] pidList;
    public double voltageComp;

    public SparkControllerInfo drive(){
        canbusUse = Usages.driveUsage;
        currentLim = Electrical.driveCurrentLim;
        invert = Setup.driveInvert;
        idleMode = IdleModes.driveIdle;
        posConversion = ConversionFactors.driveConversionPositionFactor;
        velConversion = ConversionFactors.driveConversionVelocityFactor;
        pidList = PID.drivePID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo angle(){
        canbusUse = Usages.angleUsage;
        currentLim = Electrical.angleCurrentLim;
        invert = Setup.angleInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.angleIdle;
        posConversion = ConversionFactors.angleConversionPositionFactor;
        velConversion = ConversionFactors.angleConversionVelocityFactor;
        pidList = PID.anglePID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo claw(){
        canbusUse = Usages.clawUsage;
        currentLim = Electrical.clawLim;
        invert = Setup.clawInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.clawIdle;
        pidList = PID.clawPID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo elevator(){
        canbusUse = Usages.elevatorUsage;
        currentLim = Electrical.elevatorLim;
        invert = Setup.elevatorInvert;
        alternateAbsolute = false;
        idleMode = IdleModes.elevatorIdle;
        pidList = PID.elevatorPID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo shooterWrist(){
        canbusUse = Usages.wristUsage;
        currentLim = Electrical.wristLim;
        invert = Setup.wristInvert;
        alternateAbsolute = true;
        idleMode = IdleModes.wristIdle;
        pidList = PID.wristPID;
        voltageComp = Electrical.voltageComp;
        return this;
    }
} 