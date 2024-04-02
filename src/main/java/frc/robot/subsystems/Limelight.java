package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTableInstance _instance = NetworkTableInstance.getDefault();
    private final NetworkTable _table = _instance.getTable("limelight");
    private double angle;

    public boolean isTargets(){
        if(_table.getEntry("<tv>").getDouble(0) == 1){
            return true;
        } else {
            return false;
        }
    }

    public Pose2d getRobotPoseInTargetSpace() {
        double[] robotPoseArray = new double[6];
        robotPoseArray = _table.getEntry("botpose_targetspace").getDoubleArray(robotPoseArray);
        double x = robotPoseArray[0];
        double z = robotPoseArray[2];
        double rotation = robotPoseArray[4];

        return new Pose2d(x, z, Rotation2d.fromDegrees(rotation));
    }

    public double getPivotAngletoSpeaker() {
        double[] robotPoseArray = new double[6];
        robotPoseArray = _table.getEntry("targetpose_robotspace").getDoubleArray(robotPoseArray);

        //target space from the perspective of looking at the target:
        //+X to the right of the target
        //+Y down to the ground
        //+Z straight out from the target

        // if (robotPoseArray == new double[6]) {
        //     // double x = robotPoseArray[0];
        //     double y = 82 - 16 + 4;  // Vertical Inches from pivot to top of speaker opening, +4 to combat gravity
        //     double z = robotPoseArray[2];   // Inches from camera to speaker

        //     // Did a little math, and it seems like adding the distance to our angle matches the curve of what we measured to work.
        //     angle = Math.toDegrees(Math.atan(y/(z+4))) + (z/12)-3;
        
        //     // Add flat angle
        //     angle += 60;

        //     return angle;
        // } else {
        //     return 0.0;
        // }

        angle = robotPoseArray[2];

        return angle;
    }

    public double getRobotDirectiontoSpeaker() {
        return _table.getEntry("tx").getDouble(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Aim Bot RoboPose[2]", angle);
    }
}