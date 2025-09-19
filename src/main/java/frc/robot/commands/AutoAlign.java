package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

public class AutoAlign extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final NetworkTable limelightTable;
    private final APPID drivePID;
    private final APPID turnPID;
    private final int targetTagID;
    private final double positionTolerance;
    private final double angleTolerance;
    private final double offsetDistance; // Distance to stop away from tag (meters)
    
    private boolean tagFound;
    
    // Constants - using similar values to your DriveToPoint
    private static final double kDriveP = 0.6;
    private static final double kDriveI = 0.0;
    private static final double kDriveD = 0.05;
    private static final double kMaxDriveSpeed = 0.25;
    
    private static final double kTurnP = 0.02;
    private static final double kTurnI = 0.0;
    private static final double kTurnD = 0.0;
    private static final double kMaxRotationSpeed = 1.75;
    
    /**
     * Creates a new AutoAlign command using Limelight's 3D target space tracking
     * @param driveSubsystem The drive subsystem
     * @param targetTagID The ID of the AprilTag to align to
     * @param offsetDistance Distance to stop away from the tag (meters)
     * @param positionTolerance The tolerance for position (meters)
     * @param angleTolerance The tolerance for angle (degrees)
     */
    public AutoAlign(DriveSubsystem driveSubsystem, int targetTagID, double offsetDistance, 
                    double positionTolerance, double angleTolerance) {
        
        this.driveSubsystem = driveSubsystem;
        this.targetTagID = targetTagID;
        this.offsetDistance = offsetDistance;
        this.positionTolerance = positionTolerance;
        this.angleTolerance = angleTolerance;
        
        // Initialize Limelight NetworkTable
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        this.tagFound = false;
        
        // Create PID controllers
        this.drivePID = new APPID(kDriveP, kDriveI, kDriveD, positionTolerance);
        this.drivePID.setMaxOutput(kMaxDriveSpeed);
        
        this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleTolerance);
        this.turnPID.setMaxOutput(kMaxRotationSpeed);
        this.turnPID.setDesiredValue(0.0); // Always want zero angle error
        
        addRequirements(driveSubsystem);
    }
    
    /**
     * Convenience constructor with default tolerances and offset
     */
    public AutoAlign(DriveSubsystem driveSubsystem, int targetTagID) {
        this(driveSubsystem, targetTagID, 1.0, 0.15, 3.0);
    }
    
    @Override
    public void initialize() {
        drivePID.reset();
        turnPID.reset();
        tagFound = false;
        
        // Set Limelight pipeline for AprilTag detection (usually pipeline 0)
        limelightTable.getEntry("pipeline").setNumber(0);
        
        System.out.println("AutoAlign: Looking for AprilTag ID " + targetTagID + 
                          " with " + offsetDistance + "m offset");
    }
    
    /**
     * Get robot pose in target (AprilTag) space from Limelight
     * @return Pose of robot relative to the AprilTag, or null if no valid target
     */
    private Pose getRobotPoseInTargetSpace() {
        // Check if Limelight has a valid target
        double tv = limelightTable.getEntry("tv").getDouble(0);
        if (tv == 0) {
            return null; // No target found
        }
        
        // Get the detected tag ID
        double detectedTagID = limelightTable.getEntry("tid").getDouble(-1);
        if (detectedTagID != targetTagID) {
            return null; // Wrong tag detected
        }
        
        // Get robot pose in target space from Limelight
        // targetpose_robotspace gives us [x, y, z, rx, ry, rz] where:
        // - x: right/left relative to tag (+ = right)
        // - y: up/down relative to tag (+ = up) 
        // - z: forward/back relative to tag (+ = forward/away from tag)
        // - rx, ry, rz: rotation about x, y, z axes
        double[] robotPoseInTargetSpace = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        
        if (robotPoseInTargetSpace.length >= 6) {
            // Convert from target space to field-relative coordinates
            // In target space: x=right, z=forward (away from tag)
            // We want field coordinates where robot is positioned relative to tag
            double targetSpaceX = robotPoseInTargetSpace[0]; // Right/left of tag
            double targetSpaceZ = robotPoseInTargetSpace[2]; // Forward/back from tag
            double targetSpaceRY = robotPoseInTargetSpace[4]; // Rotation about Y (yaw)
            
            // Convert target space to field space
            // If robot is at (x=1, z=2) in target space, robot is 1m right and 2m forward of tag
            return new Pose(-targetSpaceX, targetSpaceZ, -targetSpaceRY);
        }
        
        return null;
    }
    
    @Override
    public void execute() {
        // Get robot pose relative to the target AprilTag
        Pose robotInTargetSpace = getRobotPoseInTargetSpace();
        
        if (robotInTargetSpace == null) {
            // Tag not found - stop the robot and continue searching
            driveSubsystem.drive(0.0, 0.0, 0.0, true);
            tagFound = false;
            return;
        }
        
        // Tag found!
        if (!tagFound) {
            tagFound = true;
            System.out.println("AutoAlign: Tag " + targetTagID + " found! " +
                              "Robot at (" + robotInTargetSpace.GetXValue() + ", " + 
                              robotInTargetSpace.GetYValue() + ") relative to tag");
        }
        
        // === TRANSLATION CONTROL ===
        // Calculate where we want to be relative to the tag
        // Target: offsetDistance meters directly in front of the tag (y = offsetDistance, x = 0)
        double targetX = 0.0; // Centered on tag
        double targetY = offsetDistance; // Distance away from tag
        
        // Current position relative to tag
        double currentX = robotInTargetSpace.GetXValue();
        double currentY = robotInTargetSpace.GetYValue();
        
        // Calculate errors
        double xError = targetX - currentX; // Positive means move right
        double yError = targetY - currentY; // Positive means move away from tag
        
        // Set PID targets (we want zero error)
        double distanceError = Math.sqrt(xError * xError + yError * yError);
        drivePID.setDesiredValue(distanceError);
        
        // Calculate speed from PID
        double speed = drivePID.calcPID(0.0);
        
        // Calculate field-relative velocities
        // Transform target space movements to robot-relative movements
        double robotRelativeX = -xError; // Negative because target space X is flipped
        double robotRelativeY = yError;
        
        // Normalize and apply speed
        double magnitude = Math.sqrt(robotRelativeX * robotRelativeX + robotRelativeY * robotRelativeY);
        if (magnitude > 0.01) {
            robotRelativeX = (robotRelativeX / magnitude) * speed;
            robotRelativeY = (robotRelativeY / magnitude) * speed;
        } else {
            robotRelativeX = 0;
            robotRelativeY = 0;
        }
        
        // === ROTATION CONTROL ===
        // We want to face the tag (0 degrees in target space)
        double currentAngle = robotInTargetSpace.GetAngleValue();
        double targetAngle = 0.0; // Face directly at tag
        
        // Calculate shortest path angle error
        double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);
        
        // Use PID on the angle error
        double rotationOutput = -turnPID.calcPID(angleError);
        
        // Drive the robot (using field-relative control)
        driveSubsystem.drive(robotRelativeX, robotRelativeY, rotationOutput, true);
        
        // Debug output
        if (tagFound) {
            System.out.println("AutoAlign: Pos(" + 
                String.format("%.2f", currentX) + ", " + String.format("%.2f", currentY) + ") " +
                "Target(" + String.format("%.2f", targetX) + ", " + String.format("%.2f", targetY) + ") " +
                "Angle: " + String.format("%.1f", currentAngle));
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
        
        if (interrupted) {
            System.out.println("AutoAlign: Command interrupted");
        } else {
            System.out.println("AutoAlign: Successfully aligned to AprilTag " + targetTagID);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Must have found the tag first
        if (!tagFound) {
            return false;
        }
        
        // Check if tag is still visible
        double tv = limelightTable.getEntry("tv").getDouble(0);
        if (tv == 0) {
            return false; // Lost the tag, keep trying
        }
        
        // Get current pose in target space
        Pose robotInTargetSpace = getRobotPoseInTargetSpace();
        if (robotInTargetSpace == null) {
            return false;
        }
        
        // Check position tolerance
        double targetX = 0.0;
        double targetY = offsetDistance;
        double currentX = robotInTargetSpace.GetXValue();
        double currentY = robotInTargetSpace.GetYValue();
        
        double xError = Math.abs(targetX - currentX);
        double yError = Math.abs(targetY - currentY);
        double positionError = Math.sqrt(xError * xError + yError * yError);
        
        boolean positionOnTarget = positionError <= positionTolerance;
        
        // Check angle tolerance (want to face the tag)
        double currentAngle = robotInTargetSpace.GetAngleValue();
        double angleError = Math.abs(Calculations.shortestAngularDistance(0.0, currentAngle));
        boolean angleOnTarget = angleError <= angleTolerance;
        
        return positionOnTarget && angleOnTarget;
    }
    
    // Utility methods for debugging and monitoring
    
    /**
     * Check if the target AprilTag has been found
     * @return true if tag is currently detected
     */
    public boolean isTagFound() {
        return tagFound;
    }
    
    /**
     * Get current robot pose in target space
     * @return Pose relative to AprilTag, or null if not found
     */
    public Pose getRobotPoseRelativeToTag() {
        return getRobotPoseInTargetSpace();
    }
    
    /**
     * Get the current distance to target position
     * @return distance in meters, or -1 if no target
     */
    public double getDistanceToTarget() {
        Pose robotInTargetSpace = getRobotPoseInTargetSpace();
        if (robotInTargetSpace == null) {
            return -1.0;
        }
        
        double targetX = 0.0;
        double targetY = offsetDistance;
        double currentX = robotInTargetSpace.GetXValue();
        double currentY = robotInTargetSpace.GetYValue();
        
        double xError = targetX - currentX;
        double yError = targetY - currentY;
        
        return Math.sqrt(xError * xError + yError * yError);
    }
    
    /**
     * Get the current angle error relative to tag
     * @return angle error in degrees, or 999 if no target
     */
    public double getAngleError() {
        Pose robotInTargetSpace = getRobotPoseInTargetSpace();
        if (robotInTargetSpace == null) {
            return 999.0;
        }
        
        return Calculations.shortestAngularDistance(0.0, robotInTargetSpace.GetAngleValue());
    }
    
    /**
     * Check if Limelight currently sees the target tag
     * @return true if target is visible
     */
    public boolean hasValidTarget() {
        double tv = limelightTable.getEntry("tv").getDouble(0);
        double tid = limelightTable.getEntry("tid").getDouble(-1);
        return tv == 1 && tid == targetTagID;
    }
    
    /**
     * Get current X position relative to tag (right/left)
     * @return X position in meters (+ = right of tag)
     */
    public double getXRelativeToTag() {
        Pose pose = getRobotPoseInTargetSpace();
        return pose != null ? pose.GetXValue() : 0.0;
    }
    
    /**
     * Get current Y position relative to tag (forward/back)
     * @return Y position in meters (+ = away from tag)
     */
    public double getYRelativeToTag() {
        Pose pose = getRobotPoseInTargetSpace();
        return pose != null ? pose.GetYValue() : 0.0;
    }
}