package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.CANSparkMaxUtil.Usage;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {
    /* Used for Constants Used Once On Initialization of Robot or Subsystems */
    public final static class Setup {

        /* Swerve Module Ids and Constants */
        public static final int[] moduleIDs = new int[] {0, 1, 2, 3};
        public static final int[] driveMotors = new int[] {1, 3, 5, 7};
        public static final int[] angleMotors = new int[] {2, 4, 6, 8};
        public static final int[] moduleCancoders = new int[] {9, 10, 11, 12};
        public static final double[] angleOffsets = new double[] {-132.5, 143.0, 55.5, 223.0};
        public static final double gyroAngleOffset = 90.0; // If gyro is mounted at an angle, set this to fix it.

        /* Intake IDs */
        public static final int wristSpark = 16;

        public static final int clawSpark = 17;

        public static final int elevatorSpark1 = 18;
        public static final int elevatorSpark2 = 19;

        public static final int beambreakID = 0;
        // public static final int physicalSwitchID = 2;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true;

        public static final boolean clawInvert = false; // TODO - check prototype part for actual values
        public static final boolean elevatorInvert = false;
        public static final boolean wristInvert = true;


        public enum shooterInverts {
            left(true),
            right(false);
            public final boolean Invert;
            shooterInverts(boolean Invert){
                this.Invert = Invert;
            }
        }
    }

    public final static class Swerve {
        public static final double stickDeadband = 0.1;
        public static final double autoAimTolerance = 1.0;

        /* Drivetrain Calculation Constants */
        /* Input these units from center of swerve modules */
        public static final double trackWidth = Units.inchesToMeters(26.0);
        public static final double trackLength = Units.inchesToMeters(28.0);

        /* Input Current Wheel Diameter, Can Change Due To Amount Of Wear */
        public static final double wheelDiameter = Units.inchesToMeters(4); // Wheel diameter in inches
        public static final double wheelCircimference = wheelDiameter * Math.PI;

        /* Gyro Direction Toggle */
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW- (Clockwise is increasing rotation values)

        /* Cancoder Invert */
        public static final boolean canCoderInvert = false;

        /* Speed Settings */
        public static final double maxSpeed = 5.00; // meters per second
        public static final double maxAngularVelocity = 7; // radians per second (was 4.25, changed because turn speed suddenly dropped)

        /* Mk4i Module Gear Ratios */
        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (150.0 / 7.0); // 150:7
    

        /* Swerve Module Positions (Currently in solid rectangle context) */
        public static final Translation2d[] modulePositions = new Translation2d[] {
            new Translation2d( (trackLength / 2.0) - Units.inchesToMeters(2.5),  (trackWidth / 2.0) - Units.inchesToMeters(2.5)),
            new Translation2d( (trackLength / 2.0) - Units.inchesToMeters(2.5), (-trackWidth / 2.0) + Units.inchesToMeters(2.5)),
            new Translation2d((-trackLength / 2.0) + Units.inchesToMeters(2.5),  (trackWidth / 2.0) - Units.inchesToMeters(2.5)),
            new Translation2d((-trackLength / 2.0) + Units.inchesToMeters(2.5), (-trackWidth / 2.0) + Units.inchesToMeters(2.5))
        };

        /* Swerve Kinematics */
        public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            modulePositions[0],
            modulePositions[1],
            modulePositions[2],
            modulePositions[3]
        );

        // TODO - Keep a close look to this values
        // Values moved down below with other PID values to keep everything together
        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
            new PIDConstants(4.0, 0, 0.2), // Translation constants 
            new PIDConstants(1, 0, 0) // Rotation constants 
            // 2024 -> 2025 import change. Constructor simplified, deleted maxspeed, drive base radius, and replanning config
        );
    }

    public static enum Level {Retracted, Processor, L1, L2, L2A, L3, L3A, L4}
    public static enum Location {ReefLeft, ReefRight, ReefCenter, Pickup, Processor}
    
    public static final class Claw {

        // gear ratio moved to conversion factors

        /* Min/Max Speeds */
        public static final double intakeSpeed = 5;
        public static final double maxVoltage = 5;

    }

    public static final class Elevator {

        // gear ratio moved to conversion factors

        /* Min/Max Speeds */
        public static final double maxVoltage = 3;
        public static final double minVoltage = -.5;

        /* Min/Max Heights */
        public static final double minHeight = 0;       // TODO - currently in motor rotations, will probably switch to inches
        public static final double maxHeight = 40;
        
        /* setElevator height seeking tolerance */
        public static final double tolerance = 0.5; // Rename this pls

        public static final double selfDestructAngle = 62;

        public static final double gravityConstant = 0.1;

        public static final double softHeightMinimum = 1;
    }

    public static final class AutoConstants {
        
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
    }

    public static final class Wrist{

        public static final double minimumAngle = 41.0; // Coral Intake Position
        public static final double maximumAngle = 220.0; // 

        // Gear reduction moved to conversion factors
        
        public static final double maxVoltage = 6;
        public static final double maxAccel = 0.0;
        public static final double maxSpeed = 0.0;

        public static final double angleTolerance = 1.0;
        public static final double gearReduction = 0.0;
    }

    public final static class Electrical {

        /* Base 12 Volt System */
        public static final double voltageComp = 12.0;

        /* Swerve Electrical Limits */
        public static final int driveCurrentLim = 40;
        public static final int angleCurrentLim = 20;
        
        /* Subsystems */
        public static final int clawLim = 20;  // TODO - check part for actual values
        public static final int elevatorLim = 40;
        public static final int wristLim = 20;

        public static final double neoMinVoltage = 0.05; // guessed value
    }
    
    public final static class PID {

        /* Format {P, I, D, FF} 
        P, I, and D are standard PID
        FF is the inverse of kV in SVA control. Used ONLY in velocity control.
        */

        /* Swerve PIDs */
        public static final double[] drivePID = new double[] {0.3, 0.0, 0.0, 0.0};
        public static final double[] anglePID = new double[] {0.01, 0.0, 0.0, 0.0};
        
        /* Subsystems */
        public static final double[] clawPID = new double[] {0.05, 0.0, 0.0, 0.0}; // TODO - check prototype part for actual values
        public static final double[] elevatorPID = new double[] {0.25, 0.0, 0.05, 0.0};
        public static final double[] wristPID = new double[] {0.2, 0.0, 0.05, 0.0};
    }

    public final static class SVA {

        /* {Static, Velocity, Acceleration} */    /* format: Ks, Kv, Ka */
        /* Swerve */
        // public static final double[] driveMotorsSVA = new double[] {0.3, 2.55, 0.27};    // Last year's SVA values. 
        // TODO - Run SystemID to find new values for this year's bot.
        public static final double[] driveMotorsSVA = new double[] {0.2, 2.57, 0.29};

        public static final double[] ElevSVA = new double[] {0.0, 0.5, 0.01}; // TODO - sysid characterization (doing kS manually elsewhere)
        public static final double[] WristSVA = new double[] {0.0, 0.05, 0.01}; // TODO - sysid characterization
    }

    public final static class ConversionFactors {
        /* All numbers in 1 output to required input, or one wheel spin to motor spin */

        /* Swerve Drive Conversions */
        public static final double driveConversionPositionFactor = Swerve.wheelCircimference / Swerve.driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0 ; //rpm to rps
        
        public static final double angleConversionPositionFactor = 360.0 / Swerve.angleGearRatio;
        public static final double angleConversionVelocityFactor = angleConversionPositionFactor / 60.0 ; //rpm to rps
        
        /* Other Subsystem Conversions */
        // public static final double elevatorConversionPositionFactor = 1/6.4;   // 10 tooth small : 64 tooth large
        public static final double elevatorConversionPositionFactor = 1.015625;   // 10 tooth small : 64 tooth large, 1 rot : 3.25 in of chain (18 tooth sprocket), 1 inch of 1st stage : 2 inch of 2nd stage
        public static final double elevatorConversionVelocityFactor = elevatorConversionPositionFactor / 60.0; //rpm to rps
        
        // TODO - Temp smaller position factor, set real factor once we have it the right way around
        public static final double wristConversionPositionFactor = 1.0/360.0;   // 1:360 degree conversion
        public static final double wristConversionVelocityFactor = elevatorConversionPositionFactor / 60.0; //rpm to rps
        
        /* These are not good conversion factors, but they will fill the slot */
        public static final double defaultConversionPositionFactor = 1.0;
        public static final double defaultConversionVelocityFactor = defaultConversionPositionFactor / 60.0; //rpm to rps

    }

    public final static class IdleModes {
        /* Swerve Idles */
        public static final IdleMode driveIdle = IdleMode.kBrake;
        public static final IdleMode angleIdle = IdleMode.kBrake;

        public static final IdleMode clawIdle = IdleMode.kBrake;
        public static final IdleMode elevatorIdle = IdleMode.kBrake;
        public static final IdleMode wristIdle = IdleMode.kBrake;
    }

    public final static class Usages {
        /* Swerve Usages */
        public static final Usage driveUsage = Usage.kAll;
        public static final Usage angleUsage = Usage.kPositionOnly;

        public static final Usage clawUsage = Usage.kPositionOnly;
        public static final Usage elevatorUsage = Usage.kPositionOnly;
        public static final Usage wristUsage = Usage.kPositionOnly;
    }
}