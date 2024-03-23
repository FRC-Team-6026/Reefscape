package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTableInstance _instance = NetworkTableInstance.getDefault();
    private final NetworkTable _table = _instance.getTable("limelight");

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
        //target space from the perspective of looking at the target:
        //+X to the right of the target
        //+Y down to the ground
        //+Z straight out from the target
        double x = robotPoseArray[0];
        double y = 80;
        double z = robotPoseArray[2];
        double rotation = robotPoseArray[4];

        double d = Math.sqrt((x * x) + (z * z));

        return new Pose2d(x, z, Rotation2d.fromDegrees(rotation));
    }

    public double getPivotAngletoSpeaker() {
        double[] robotPoseArray = new double[6];
        robotPoseArray = _table.getEntry("targetpose_robotspace").getDoubleArray(robotPoseArray);
        double x = robotPoseArray[0];
        double z = 80; // TODO - subtract the height of the pivot

        return Math.toDegrees(Math.atan(z/x));
    }

    public double getRobotDirectiontoSpeaker() {
        return _table.getEntry("tx").getDouble(0);
    }
}