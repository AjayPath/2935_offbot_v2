package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.APPID;

public class AutoAlign extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final APPID yController;
    private final String limelightName;
    private final Timer dontSeeTagTimer;
    private final Timer alignedTimer;
    private double targetTagID = -1;
    
    // PID Constants for Y alignment (left/right movement)
    private static final double kYP = 0.8;  // Adjust based on your robot
    private static final double kYI = 0.0;
    private static final double kYD = 0.0;
    private static final double kMaxYSpeed = 0.5;  // Maximum strafe speed
    
    // Tolerances and timing
    private static final double Y_SETPOINT = 0.0;  // Want to be centered (Y = 0)
    private static final double Y_TOLERANCE = 0.1;  // Meters
    private static final double DONT_SEE_TAG_TIMEOUT = 2.0;  // seconds
    private static final double ALIGNED_TIME = 0.3;  // seconds to stay aligned
    
    /**
     * Creates a new AlignToTagY command
     * @param driveSubsystem The drive subsystem to use
     * @param limelightName The name of the Limelight ("" for default)
     */
    public AutoAlign(DriveSubsystem driveSubsystem, String limelightName) {
        this.driveSubsystem = driveSubsystem;
        this.limelightName = limelightName;
        
        // Create PID controller for Y (left/right) movement
        this.yController = new APPID(kYP, kYI, kYD, Y_TOLERANCE);
        this.yController.setMaxOutput(kMaxYSpeed);
        
        // Create timers
        this.dontSeeTagTimer = new Timer();
        this.alignedTimer = new Timer();
        
        addRequirements(driveSubsystem);
    }
    
    /**
     * Convenience constructor using default Limelight
     */
    public AutoAlign(DriveSubsystem driveSubsystem) {
        this(driveSubsystem, "limelight-l");
    }
    
    @Override
    public void initialize() {
        // Reset PID controller
        yController.reset();
        yController.setDesiredValue(Y_SETPOINT);
        
        // Start timers
        dontSeeTagTimer.restart();
        alignedTimer.restart();
        
        // Remember which tag we're targeting (first one we see)
        if (LimelightHelpers.getTV(limelightName)) {
            targetTagID = LimelightHelpers.getFiducialID(limelightName);
            System.out.println("AlignToTagY: Targeting tag ID " + targetTagID);
        } else {
            targetTagID = -1;
            System.out.println("AlignToTagY: No tag visible at start");
        }
    }
    
    @Override
    public void execute() {
        boolean canSeeTag = LimelightHelpers.getTV(limelightName);
        double currentTagID = LimelightHelpers.getFiducialID(limelightName);
        
        // Check if we can see our target tag
        boolean hasValidTarget = canSeeTag && 
                                (targetTagID == -1 || currentTagID == targetTagID);
        
        if (hasValidTarget) {
            // Reset the "don't see tag" timer since we can see it
            dontSeeTagTimer.restart();
            
            // Update target tag ID if we didn't have one
            if (targetTagID == -1) {
                targetTagID = currentTagID;
            }
            
            // Get robot position relative to tag
            double[] positions = LimelightHelpers.getBotPose_TargetSpace(limelightName);
            
            if (positions.length >= 6) {
                // positions[0] is the Y position relative to tag (left/right)
                double yPosition = positions[0];
                
                // Calculate PID output for Y movement (strafe left/right)
                double ySpeed = -yController.calcPID(yPosition);
                
                // Drive only in Y direction (strafe)
                driveSubsystem.drive(0.0, ySpeed, 0.0, true);
                
                // Check if we're aligned
                boolean yAligned = Math.abs(yPosition - Y_SETPOINT) <= Y_TOLERANCE;
                
                if (yAligned) {
                    // We're aligned, let the timer run
                } else {
                    // Not aligned, reset the aligned timer
                    alignedTimer.restart();
                }
                
                // Display debug info
                SmartDashboard.putBoolean("AlignY/Has Target", true);
                SmartDashboard.putNumber("AlignY/Tag ID", targetTagID);
                SmartDashboard.putNumber("AlignY/Y Position", yPosition);
                SmartDashboard.putNumber("AlignY/Y Speed", ySpeed);
                SmartDashboard.putBoolean("AlignY/Y Aligned", yAligned);
                SmartDashboard.putNumber("AlignY/Aligned Time", alignedTimer.get());
                
            } else {
                // No position data available
                driveSubsystem.drive(0.0, 0.0, 0.0, true);
                SmartDashboard.putString("AlignY/Status", "No position data");
            }
            
        } else {
            // No valid target, stop moving
            driveSubsystem.drive(0.0, 0.0, 0.0, true);
            SmartDashboard.putBoolean("AlignY/Has Target", false);
            SmartDashboard.putString("AlignY/Status", "No valid target");
        }
        
        // Always show timer info
        SmartDashboard.putNumber("AlignY/No Tag Timer", dontSeeTagTimer.get());
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
        
        if (interrupted) {
            System.out.println("AlignToTagY: Command interrupted");
        } else {
            System.out.println("AlignToTagY: Successfully aligned to tag");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish if we haven't seen the tag for too long
        boolean tagTimeout = dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_TIMEOUT);
        
        // Or if we've been aligned for the required time
        boolean stayAligned = alignedTimer.hasElapsed(ALIGNED_TIME);
        
        return tagTimeout || stayAligned;
    }
    
    /**
     * Get the current Y position error
     * @return Y position relative to tag in meters
     */
    public double getYError() {
        if (LimelightHelpers.getTV(limelightName)) {
            double[] positions = LimelightHelpers.getBotPose_TargetSpace(limelightName);
            if (positions.length >= 6) {
                return positions[0] - Y_SETPOINT;
            }
        }
        return 0.0;
    }
}