package frc.lib.configs.Sparkmax;

import com.revrobotics.CANSparkBase.IdleMode;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.*;
import frc.robot.Constants.Setup.shooterInverts;

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
        currentLim = Electical.driveCurrentLim;
        invert = Setup.driveInvert;
        idleMode = IdleModes.driveIdle;
        posConversion = ConversionFactors.driveConversionPositionFactor;
        velConversion = ConversionFactors.driveConversionVelocityFactor;
        pidList = PID.drivePID;
        voltageComp = Electical.voltageComp;
        return this;
    }

    public SparkControllerInfo angle(){
        canbusUse = Usages.angleUsage;
        currentLim = Electical.angleCurrentLim;
        invert = Setup.angleInvert;
        idleMode = IdleModes.angleIdle;
        posConversion = ConversionFactors.angleConversionPositionFactor;
        velConversion = ConversionFactors.angleConversionVelocityFactor;
        pidList = PID.anglePID;
        voltageComp = Electical.voltageComp;
        return this;
    }

    public SparkControllerInfo shooterWheel(shooterInverts shooterInvert){
        canbusUse = Usages.shooterWheels;
        currentLim = Electical.shooterWheelCurrentLim;
        invert = shooterInvert.Invert;
        idleMode = IdleModes.shooterWheels;
        posConversion = ConversionFactors.shooterBaseConversionFactor;
        velConversion = ConversionFactors.shooterBaseConversionFactor/60;
        pidList = PID.shooterWheelsPID;
        voltageComp = Electical.voltageComp;
        return this;
    }

    public SparkControllerInfo intake(){
        canbusUse = Usages.intakeRoller;
        currentLim = Electical.intakeRollerCurrentLim;
        invert = Setup.intakeInvert;
        idleMode = IdleModes.intakeRoller;
        posConversion = ConversionFactors.intakeBaseConversionFactor;
        velConversion = ConversionFactors.intakeBaseVelocityConversionFactor;
        pidList = PID.intakeRollerPID;
        voltageComp = Electical.voltageComp;
        return this;
    }

}
