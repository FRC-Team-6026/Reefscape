package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTableInstance _instance = NetworkTableInstance.getDefault();
    private NetworkTable _table;

    public Limelight(String networkTableName) {
        _table = _instance.getTable(networkTableName);
    }

    public boolean isTargets(){
        if(_table.getEntry("tv").getDouble(0) > 0.1){
            return true;
        } else {
            return false;
        }
    }

      public double getRobotRotationtoSpeaker() {
        double val = _table.getEntry("tx").getDouble(0);
        return val;
    }
    
    public Pose2d getRobotPoseInTargetSpace() {
        double[] robotPoseArray = new double[6];
        robotPoseArray = _table.getEntry("botpose_targetspace").getDoubleArray(robotPoseArray);
        double x = robotPoseArray[0];
        double z = robotPoseArray[2];
        double rotation = robotPoseArray[4];

        return new Pose2d(x, z, Rotation2d.fromDegrees(rotation));
    }

}