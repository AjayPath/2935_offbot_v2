package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

public class DriveToPoint extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final APPID drivePID;
    private final APPID turnPID;
    private final Pose targetPose;
    private final double positionTolerance;
    private final double angleTolerance;
    
    // Constants
    private static final double kDriveP = 0.6;
    private static final double kDriveI = 0.0;
    private static final double kDriveD = 0.05;
    private static final double kMaxDriveSpeed = 1;
    
    private static final double kTurnP = 0.02;
    private static final double kTurnI = 0.0;
    private static final double kTurnD = 0.0;
    private static final double kMaxRotationSpeed = 1.75;
    
    /**
     * Creates a new DrivePointCommand
     * @param driveSubsystem The drive subsystem
     * @param targetPose The target pose (x, y in meters, angle in degrees)
     * @param positionTolerance The tolerance for position (meters)
     * @param angleTolerance The tolerance for angle (degrees)
     */
    public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, double targetAngle, double positionTolerance, double angleTolerance) {
        
        this.driveSubsystem = driveSubsystem;
        this.targetPose = new Pose(targetX, targetY, targetAngle);
        this.positionTolerance = positionTolerance;
        this.angleTolerance = angleTolerance;
        
        // Create PID controllers
        this.drivePID = new APPID(kDriveP, kDriveI, kDriveD, positionTolerance);
        this.drivePID.setMaxOutput(kMaxDriveSpeed);
        
        this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleTolerance);
        this.turnPID.setMaxOutput(kMaxRotationSpeed);
        this.turnPID.setDesiredValue(0.0); // Always want zero angle error
        
        addRequirements(driveSubsystem);
    }
    
    /**
     * Convenience constructor with default tolerances
     */
    public DriveToPoint(DriveSubsystem driveSubsystem, double targetX, double targetY, double targetAngle) {
        this(driveSubsystem, targetX, targetY, targetAngle, 0.1, 2.0);
    }
    
    @Override
    public void initialize() {
        drivePID.reset();
        turnPID.reset();
        System.out.println("DrivePoint: Targeting (" + 
                          targetPose.GetXValue() + ", " + 
                          targetPose.GetYValue() + ", " + 
                          targetPose.GetAngleValue() + ")");
    }
    
    @Override
    public void execute() {
        // Get current position
        Pose currentPose = driveSubsystem.getCustomPose();
        
        // === TRANSLATION CONTROL ===
        // Calculate vector to target position
        Vector difference = targetPose.Subtract(currentPose);
        double distanceToTarget = difference.GetMag();
        
        // Set the target distance to current distance (so PID drives it to 0)
        drivePID.setDesiredValue(distanceToTarget);
        
        // Calculate PID output for translation speed
        double speed = drivePID.calcPID(0.0);
        
        // Calculate x and y velocities
        double angleToTargetRad = difference.GetAngle().getRadians();
        double xVel = speed * Math.cos(angleToTargetRad);
        double yVel = speed * Math.sin(angleToTargetRad);
        
        // === ROTATION CONTROL ===
        // Get current angle and normalize to [0, 360) range
        double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
        double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
        
        // Calculate shortest path angle error
        double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);
        
        // Use PID on the angle error
        double rotationOutput = -turnPID.calcPID(angleError);
        
        // Drive the robot with both translation and rotation
        driveSubsystem.drive(xVel, yVel, rotationOutput, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
        
        if (interrupted) {
            System.out.println("DrivePoint: Command interrupted");
        } else {
            // Update robot pose to target pose when command completes successfully
            driveSubsystem.setOdom(targetPose.GetXValue(), targetPose.GetYValue(), targetPose.GetAngleValue());
            System.out.println("DrivePoint: Command completed - Pose updated to target");
        }
    }
    
    @Override
    public boolean isFinished() {
        Pose currentPose = driveSubsystem.getCustomPose();
        
        // Check position tolerance
        double distanceToTarget = targetPose.Subtract(currentPose).GetMag();
        boolean positionOnTarget = distanceToTarget <= positionTolerance;
        
        // Check angle tolerance
        double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
        double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
        double angleError = Math.abs(Calculations.shortestAngularDistance(targetAngle, currentAngle));
        boolean angleOnTarget = angleError <= angleTolerance;
        
        return positionOnTarget && angleOnTarget;
    }
    
    /**
     * Get the current distance to target position
     * @return distance in meters
     */
    public double getDistanceToTarget() {
        Pose currentPose = driveSubsystem.getCustomPose();
        return targetPose.Subtract(currentPose).GetMag();
    }
    
    /**
     * Get the current angle error
     * @return angle error in degrees
     */
    public double getAngleError() {
        Pose currentPose = driveSubsystem.getCustomPose();
        double currentAngle = Calculations.NormalizeAngle360(currentPose.GetAngleValue());
        double targetAngle = Calculations.NormalizeAngle360(targetPose.GetAngleValue());
        return Calculations.shortestAngularDistance(targetAngle, currentAngle);
    }
}