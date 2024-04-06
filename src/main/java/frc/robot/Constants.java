package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.CANSparkMaxUtil.Usage;

public final class Constants {
    /* Used for Constants Used Once On Initialization of Robot or Subsystems */
    public final static class Setup {

        /* Swerve Module Ids and Constants */
        public static final int[] moduleIDs = new int[] {0, 1, 2, 3};
        public static final int[] driveMotors = new int[] {1, 3, 5, 7};
        public static final int[] angleMotors = new int[] {2, 4, 6, 8};
        public static final int[] moduleCancoders = new int[] {9, 10, 11, 12};
        public static final double[] angleOffsets = new double[] {-136.1, 154.5, 157.5, -107.9};
        public static final double[] xposition = new double[] {45, 45, -45, -45};

        /* Shooter IDs */
        public static final int leftWheel = 14;
        public static final int rightWheel = 15;
        public static final int feedRoller = 16;

        /* Intake IDs */
        public static final int topRoller = 17;
        public static final int bottomRoller = 18;

        /* Pivot */
        public static final int pivotMotor = 19;

        /* Elevator */
        public static final int elevatorMotor = 20;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true; //Set false for MK4 modules
        public enum shooterInverts {
            left(true),
            right(false);
            public final boolean Invert;
            shooterInverts(boolean Invert){
                this.Invert = Invert;
            }
        }

        /* Shooter Assembly Motor Direction */
        public static final boolean feederInvert = false;
        public static final boolean intakeInvert = false;
        public static final boolean pivotInvert = false;
        public static final boolean elevatorInvert = true;
    }

    public final static class Swerve {
        public static final double stickDeadband = 0.1;
        public static final double autoAimTolerance = 1.0;

        /* Drivetrain Calculation Constants */
        /* Input these units from center of swerve modules */
        public static final double trackWidth = Units.inchesToMeters(26);
        public static final double trackLength = Units.inchesToMeters(31);

        /* Input Current Wheel Diameter, Can Change Due To Amount Of Wear */
        public static final double wheelDiameter = Units.inchesToMeters(4); // Wheel diameter in inches (should be 4 inches, testing bigger value)
        public static final double wheelCircimference = wheelDiameter * Math.PI;

        /* Gyro Direction Toggle */
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW- (Clockwise is increasing rotation values)

        /* Cancoder Invert */
        public static final boolean canCoderInvert = true;

        /* Speed Settings */
        public static final double maxSpeed = 5.00; // meters per second
        public static final double maxAngularVelocity = 7; // radians per second (was 4.25, changed because turn speed suddenly dropped)

        /* Mk4i Module Gear Ratios */
        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (150.0 / 7.0); // 150:7
    

        /* Swerve Module Positions (Currently in solid rectangle context) */
        public static final Translation2d[] modulePositions = new Translation2d[] {     // I found values being subtracted from the corners of the robot, and im assuming those values should have been in inches
            new Translation2d((trackLength / 2.0) - Units.inchesToMeters(8.5), (trackWidth / 2.0) - Units.inchesToMeters(2.5)),
            new Translation2d((trackLength / 2.0) - Units.inchesToMeters(8.5), (-trackWidth / 2.0) + Units.inchesToMeters(2.5)),
            new Translation2d((-trackLength / 2.0) + Units.inchesToMeters(2.5), (trackWidth / 2.0) - Units.inchesToMeters(2.5)),
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
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(4.0, 0, 0.2), // Translation constants 
            new PIDConstants(1, 0, 0), // Rotation constants 
            maxSpeed, 
            modulePositions[0].getNorm(), // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
        );
    }

    public static final class Shooter {

        /* Gear Ratios */
        public static final double flywheelReduction = 24.0/18.0; // Gear ratio of the spinning motor to shaft output

        // Speaker, amp and long shots voltage
        public static final double speakershotVoltage = 8;      // These values might get cut in favor of one single value, especially ampshot since we don't think we can manage that.
        public static final double ampshotVoltage = 2;
        public static final double longshotVoltage = 9;

        /* Min/Max Speeds */
        public static final double minVoltage = 1;
        public static final double maxVoltage = 12;

    }

    public static final class Feeder {

        /* Gear Ratios */
        public static final double feederWheelReduction = 24.0/20.0; // Mirrored gears control both rollers; roller gear is 24 teeth, inner is 20.

        /* Feeder Constant values */
        public static final double feederRadius = 1.25;     //TODO - Get actual constant
        public static final double feederCircumferenceInch = 1.25 * Math.PI;
        public static final double feederCircumferenceMeter = feederCircumferenceInch * 0.0254;

        /* Min/Max Speeds */
        public static final double feederPower = 1.5;
        public static final double maxVoltage = 2;

    }
    
    public static final class Intake {

        /* Gear Ratios */
        public static final double intakeRollerReduction = 24.0/11.0; //TODO - get the actual gear ratios

        /* Intake Constant values */
        public static final double rollerRadius = 1.25;
        public static final double rollerCircumferenceInch = 1.25 * Math.PI;
        public static final double rollerCircumferenceMeter = rollerCircumferenceInch * 0.0254;

        /* Min/Max Speeds */
        public static final double intakeSpeed = 5;
        public static final double maxVoltage = 5;

    }

    public static final class Pivot {

        //Absolute Encoder angle values. its no longer being a butt
        public static final int intakeAngle = 175;
        public static final int backwardsShotAngle = 186;
        public static final int forwardsShotAngle = 120;
        public static final int minimumAngle = 94;
        public static final int maximumAngle = 190;
        
        /* Gear Ratios */
        public static final double gearReduction = 24.0/11.0; // TODO - get the actual gear ratios

        /* Pivot Constant values */
        //public static final double maxSpeed = Swerve.maxSpeed * 0.5;
        public static final double maxVoltage = 3.5;

        /* Min/Max Speeds */
        public static final double maxTurnSpeed = 200;   // in deg/s
        public static final double maxAccel = 400;   // in deg/s/s

        public static final double angleTolerance = 0.5;  // tolerance (in degrees) for commands that set the pivot to an angle

    }

      public static final class Elevator {

        /* Gear Ratios */
        public static final double elevatorReduction = 9.0/1.0; // TODO - 45:1 ??? (Ask Alex lol)

        /* Min/Max Speeds */
        public static final double maxVoltage = 5.5;

        /* Deploy Positions */
        public static final double stowedPosition = 0;
        public static final double deployedPosition = 90;

    }

    public static final class AutoConstants {
        
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
    }

    public final static class Electical {

        /* Base 12 Volt System */
        public static final double voltageComp = 12.0;

        /* Swerve Electrical Limits */
        public static final int driveCurrentLim = 40;
        public static final int angleCurrentLim = 20;

        /* Shooter Electrical Limits */
        public static final int feederCurrentLim = 40;

        /* Shooter Electrical Limits */
        public static final int shooterWheelCurrentLim = 40;

        /* Intake Electrical Limits */
        public static final int intakeRollerCurrentLim = 40;

        /* Pivot Electrical Limits */
        public static final int pivotCurrentLim = 20;

        /* Elevator Electrical Limits */
        public static final int elevatorCurrentLim = 40;
    }
    
    public final static class PID {

        /* Format {P, I, D, FF} */

        /* Swerve PIDs */
        public static final double[] drivePID = new double[] {0.3, 0.0, 0.0, 0.0};
        public static final double[] anglePID = new double[] {0.01, 0.0, 0.0, 0.0};

        /* Shooter assembly PIDs */
        public static final double[] shooterWheelsPID = new double[] {0.1, 0.0, 0.0, 0.0};
        public static final double[] intakeRollerPID = new double[] {0.02, 0.0, 0.0, 0.0};
        public static final double[] feederRollerPID = new double[] {0.02, 0.0, 0.0, 0.0};
        public static final double[] pivotPID = new double[] {0.08, 0.0, 0.0, 0.0};
        public static final double[] elevatorPID = new double[] {0.02, 0.0, 0.0, 0.0};

    }

    public final static class SVA {

        /* {Static, Velocity, Acceleration} */    /* format: Ks, Kv, Ka */
        /* Swerve */
        // public static final double[] driveMotorsSVA = new double[] {0.3, 2.55, 0.27};    // Last year's SVA values. 
        //TODO - Talk to MR.Mills about changing the new calculated Ka value wich is 0.272380514 or 0.25 to 0.30
        public static final double[] driveMotorsSVA = new double[] {0.2, 2.57, 0.29};         // TODO - Testing these values, set them back if it causes issues with teleop
        
        /* Shooter Wheels*/
        // TODO - keep an eye on this SVA values 
        // flywheels should have a little resistance to being spun up, but should maintain speed easily. We want them to accelerate quickly
        public static final double[] ShooterWheelsSVA = new double[] {0.1, 0.01, 0.2};
        
        /* Shooter Wheels*/
        // very low Kv because I feed a large number to it (deg/s)
        // doing Ks manually because of issues getting to final position
        public static final double[] PivotSVA = new double[] {0.0, 0.015, 0.0};
    }

    public final static class ConversionFactors {
        /* All numbers in 1 output to required input, or one wheel spin to motor spin */

        /* Swerve Drive Conversions */
        public static final double driveConversionPositionFactor = Swerve.wheelCircimference / Swerve.driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60 ; //rpm to rps
        
        public static final double angleConversionPositionFactor = 360.0 / Swerve.angleGearRatio;
        public static final double angleConversionVelocityFactor = angleConversionPositionFactor / 60 ; //rpm to rps

        /* Shooter Conversions */
        public static final double shooterBaseConversionFactor = 1/Shooter.flywheelReduction;
        public static final double shooterBaseVelocityConversionFactor = shooterBaseConversionFactor/60;

    }

    public final static class IdleModes {
        /* Swerve Idles */
        public static final IdleMode driveIdle = IdleMode.kBrake;
        public static final IdleMode angleIdle = IdleMode.kBrake;

        /* Shooter Assembly Idle Modes */
        public static final IdleMode feeder = IdleMode.kCoast;
        public static final IdleMode shooterWheels = IdleMode.kCoast;
        public static final IdleMode intakeRoller = IdleMode.kCoast;
        public static final IdleMode shooterPivot = IdleMode.kBrake;

        /* Elevator Idle Modes */
        public static final IdleMode elevatorMotor = IdleMode.kBrake;
    }

    public final static class Usages {
        /* Swerve Usages */
        public static final Usage driveUsage = Usage.kAll;
        public static final Usage angleUsage = Usage.kPositionOnly;

        /* Shooter Assembly Usages */
        public static final Usage feeder = Usage.kVelocityOnly;
        public static final Usage shooterWheels = Usage.kVelocityOnly;
        public static final Usage intakeRoller = Usage.kVelocityOnly;
        public static final Usage shooterPivot = Usage.kAll;

        /* Elevator Motor */
        public static final Usage elevatorMotor = Usage.kPositionOnly;
    }
}
