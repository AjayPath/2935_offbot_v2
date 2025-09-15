package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.LimelightHelpers;

/**
 * Simple command to read AprilTag data and display on SmartDashboard
 */
public class LimelightTestCommand extends Command {
    
    /**
     * Creates a new ReadAprilTagDistanceCommand using default Limelight
     */
    public LimelightTestCommand() {
        // No subsystem requirements needed
    }

    @Override
    public void initialize() {
        System.out.println("AprilTag Reader Started");
    }

    @Override
    public void execute() {
        // Check if we can see a tag (same as your working example)
        boolean canSeeTag = LimelightHelpers.getTV("limelight-l");
        double tagID = LimelightHelpers.getFiducialID("limelight-l");
        
        // Put basic info on dashboard
        SmartDashboard.putBoolean("AprilTag/Can See Tag", canSeeTag);
        SmartDashboard.putNumber("AprilTag/Tag ID", tagID);
        
        // If we can see a tag, get the distance data
        if (canSeeTag && tagID > 0) {
            // Get the raw position array (same method as your example)
            double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-l");
            
            if (positions.length >= 6) {
                // Display the raw position data
                SmartDashboard.putNumber("AprilTag/X Position", positions[0]);
                SmartDashboard.putNumber("AprilTag/Y Position", positions[1]);
                SmartDashboard.putNumber("AprilTag/Z Position", positions[2]);
                
                // Calculate 2D distance (X and Y only)
                double distance2D = Math.sqrt(positions[0] * positions[0] + positions[1] * positions[1]);
                SmartDashboard.putNumber("AprilTag/2D Distance", distance2D);
                
                // Console output
                System.out.printf("Tag ID: %.0f, X: %.2f, Y: %.2f, Distance: %.2f%n", 
                                tagID, positions[0], positions[1], distance2D);
            } else {
                SmartDashboard.putString("AprilTag/Status", "Tag seen but no position data");
                System.out.println("Tag detected but position array is empty");
            }
        } else {
            // No tag detected
            SmartDashboard.putString("AprilTag/Status", "No tag detected");
            SmartDashboard.putNumber("AprilTag/X Position", 0);
            SmartDashboard.putNumber("AprilTag/Y Position", 0);
            SmartDashboard.putNumber("AprilTag/Z Position", 0);
            SmartDashboard.putNumber("AprilTag/2D Distance", 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AprilTag Reader Stopped");
    }

    @Override
    public boolean isFinished() {
        // Run continuously
        return false;
    }
}