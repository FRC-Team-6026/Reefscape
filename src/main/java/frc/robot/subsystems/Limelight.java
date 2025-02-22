package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
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

    /**
     * Uses AprilTags in view to update the robot's position, with Limelight's MegaTag (or MegaTag 2).
     * If the pose is not likely accurate, this function does nothing. Otherwise, it updates the pose.
     * 
     * @param swerve the swerve system to send the updated pose to
     * @return true if update was successful, otherwise false
     */
    public boolean updatePose(Swerve swerve) {
        double yaw = swerve.getGyro().getYaw();
        PoseEstimate limelightMeasurement;
        boolean doRejectUpdate = false;

        SetRobotOrientation("limelight", yaw, 0, 0, 0, 0, 0);

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            limelightMeasurement = getBotPoseEstimate("limelight", "botpose_orb_wpired", true);
        } else {
            limelightMeasurement = getBotPoseEstimate("limelight", "botpose_orb_wpiblue", true);
        }

        if(Math.abs(swerve.getGyro().getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(limelightMeasurement.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            swerve.resetOdometry(limelightMeasurement.pose);
        }

        return !doRejectUpdate;
    }

    

    /* Everything from this line down is copied over or lightly edited from the LimelightHelpers class provided by Limelight. 
     * We shouldn't need to interact with most of it except for the SetRobotOrientation and getBotPoseEstimate methods.
     */
    
    
     

     private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

     /**
      * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
      */
     public static class RawFiducial {
         public int id = 0;
         public double txnc = 0;
         public double tync = 0;
         public double ta = 0;
         public double distToCamera = 0;
         public double distToRobot = 0;
         public double ambiguity = 0;
 
 
         public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
             this.id = id;
             this.txnc = txnc;
             this.tync = tync;
             this.ta = ta;
             this.distToCamera = distToCamera;
             this.distToRobot = distToRobot;
             this.ambiguity = ambiguity;
         }
     }
 
      
     /**
      * Represents a 3D Pose Estimate.
      */
     public static class PoseEstimate {
         public Pose2d pose;
         public double timestampSeconds;
         public double latency;
         public int tagCount;
         public double tagSpan;
         public double avgTagDist;
         public double avgTagArea;
 
         public RawFiducial[] rawFiducials; 
         public boolean isMegaTag2;
 
         /**
          * Instantiates a PoseEstimate object with default values
          */
         public PoseEstimate() {
             this.pose = new Pose2d();
             this.timestampSeconds = 0;
             this.latency = 0;
             this.tagCount = 0;
             this.tagSpan = 0;
             this.avgTagDist = 0;
             this.avgTagArea = 0;
             this.rawFiducials = new RawFiducial[]{};
             this.isMegaTag2 = false;
         }
 
         public PoseEstimate(Pose2d pose, double timestampSeconds, double latency, 
             int tagCount, double tagSpan, double avgTagDist, 
             double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {
 
             this.pose = pose;
             this.timestampSeconds = timestampSeconds;
             this.latency = latency;
             this.tagCount = tagCount;
             this.tagSpan = tagSpan;
             this.avgTagDist = avgTagDist;
             this.avgTagArea = avgTagArea;
             this.rawFiducials = rawFiducials;
             this.isMegaTag2 = isMegaTag2;
         }
 
     }
 
     /**
      * Takes a 6-length array of pose data and converts it to a Pose2d object.
      * Uses only x, y, and yaw components, ignoring z, roll, and pitch.
      * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
      * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
      * @return Pose2d object representing the pose, or empty Pose2d if invalid data
      */
     public static Pose2d toPose2D(double[] inData){
         if(inData.length < 6)
         {
             //System.err.println("Bad LL 2D Pose Data!");
             return new Pose2d();
         }
         Translation2d tran2d = new Translation2d(inData[0], inData[1]);
         Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
         return new Pose2d(tran2d, r2d);
     }
 
     static final String sanitizeName(String name) {
         if (name == "" || name == null) {
             return "limelight";
         }
         return name;
     }
 
     public static void Flush() {
         NetworkTableInstance.getDefault().flush();
     }
 
     public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
         return getLimelightNTTable(tableName).getEntry(entryName);
     }
 
     public static NetworkTable getLimelightNTTable(String tableName) {
         return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
     }
 
     public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
         String key = tableName + "/" + entryName;
         return doubleArrayEntries.computeIfAbsent(key, k -> {
             NetworkTable table = getLimelightNTTable(tableName);
             return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
         });
     }
 
     private static double extractArrayEntry(double[] inData, int position){
         if(inData.length < position+1) { return 0; }
         return inData[position];
     }
 
 
     /**
      * Sets robot orientation values used by MegaTag2 localization algorithm.
      * 
      * @param limelightName Name/identifier of the Limelight
      * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
      * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
      * @param pitch (Unnecessary) Robot pitch in degrees 
      * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
      * @param roll (Unnecessary) Robot roll in degrees
      * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
      */
     public static void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
         double pitch, double pitchRate, 
         double roll, double rollRate) {
         SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
     }
 
     public static void SetRobotOrientation_NoFlush(String limelightName, double yaw, double yawRate, 
         double pitch, double pitchRate, 
         double roll, double rollRate) {
         SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, false);
     }
 
     private static void SetRobotOrientation_INTERNAL(String limelightName, double yaw, double yawRate, 
         double pitch, double pitchRate, 
         double roll, double rollRate, boolean flush) {
 
         double[] entries = new double[6];
         entries[0] = yaw;
         entries[1] = yawRate;
         entries[2] = pitch;
         entries[3] = pitchRate;
         entries[4] = roll;
         entries[5] = rollRate;
         getLimelightNTTableEntry(limelightName, "robot_orientation_set").setDoubleArray(entries);
         if(flush)
         {
             Flush();
         }
     }
 
     private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName, boolean isMegaTag2) {
         DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName);
         
         TimestampedDoubleArray tsValue = poseEntry.getAtomic();
         double[] poseArray = tsValue.value;
         long timestamp = tsValue.timestamp;
         
         if (poseArray.length == 0) {
             // Handle the case where no data is available
             return null; // or some default PoseEstimate
         }
     
         var pose = toPose2D(poseArray);
         double latency = extractArrayEntry(poseArray, 6);
         int tagCount = (int)extractArrayEntry(poseArray, 7);
         double tagSpan = extractArrayEntry(poseArray, 8);
         double tagDist = extractArrayEntry(poseArray, 9);
         double tagArea = extractArrayEntry(poseArray, 10);
         
         // Convert server timestamp from microseconds to seconds and adjust for latency
         double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
     
         RawFiducial[] rawFiducials = new RawFiducial[tagCount];
         int valsPerFiducial = 7;
         int expectedTotalVals = 11 + valsPerFiducial * tagCount;
     
         if (poseArray.length != expectedTotalVals) {
             // Don't populate fiducials
         } else {
             for(int i = 0; i < tagCount; i++) {
                 int baseIndex = 11 + (i * valsPerFiducial);
                 int id = (int)poseArray[baseIndex];
                 double txnc = poseArray[baseIndex + 1];
                 double tync = poseArray[baseIndex + 2];
                 double ta = poseArray[baseIndex + 3];
                 double distToCamera = poseArray[baseIndex + 4];
                 double distToRobot = poseArray[baseIndex + 5];
                 double ambiguity = poseArray[baseIndex + 6];
                 rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
             }
         }
     
         return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
     }

}