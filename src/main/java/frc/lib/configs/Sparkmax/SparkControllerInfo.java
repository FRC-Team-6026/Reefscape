package frc.lib.configs.Sparkmax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.*;

public class SparkControllerInfo {
    public Usage canbusUse;
    public int currentLim;
    public boolean invert;
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
        idleMode = IdleModes.angleIdle;
        posConversion = ConversionFactors.angleConversionPositionFactor;
        velConversion = ConversionFactors.angleConversionVelocityFactor;
        pidList = PID.anglePID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo algaeIntake(){
        canbusUse = Usages.algaeUsage;
        currentLim = Electrical.algaeLim;
        invert = Setup.algaeInvert;
        idleMode = IdleModes.algaeIdle;
        pidList = PID.algaePID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo coralIntake(){
        canbusUse = Usages.coralUsage;
        currentLim = Electrical.coralLim;
        invert = Setup.coralInvert;
        idleMode = IdleModes.coralIdle;
        pidList = PID.coralPID;
        voltageComp = Electrical.voltageComp;
        return this;
    }

    public SparkControllerInfo elevator(){
        canbusUse = Usages.elevatorUsage;
        currentLim = Electrical.elevatorLim;
        invert = Setup.elevatorInvert;
        idleMode = IdleModes.elevatorIdle;
        pidList = PID.elevatorPID;
        voltageComp = Electrical.voltageComp;
        return this;
    }
}
