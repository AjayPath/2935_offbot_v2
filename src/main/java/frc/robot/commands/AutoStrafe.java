// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.APPID;

public class AutoStrafe extends Command {
    private final DriveSubsystem driveSubsystem;
    private final APPID strafePID;
    private final Timer noTagTimer;
    private final Timer stabilityTimer;
    private final boolean isRightScore;
    private double targetTagID = -1;
    
    // Constants - you should move these to a Constants class
    private static final double STRAFE_P = 0.7;
    private static final double STRAFE_I = 0.0;
    private static final double STRAFE_D = 0.0;
    private static final double MAX_STRAFE_SPEED = 0.5;
    private static final double X_SETPOINT = 0.15; // Base X position relative to tag (meters)
    private static final double X_TOLERANCE = 0.1; // Tolerance for PID (increased from 0.02)
    private static final double NO_TAG_TIMEOUT = 2.0; // Seconds to wait without seeing tag
    private static final double STABILITY_TIME = 0.2; // Time to stay on target before finishing
    
    // Finishing tolerances
    private static final double FINISH_TOLERANCE = 0.1; // Distance from target for finishing (increased)

    public AutoStrafe(DriveSubsystem driveSubsystem, boolean isRightScore) {
        this.driveSubsystem = driveSubsystem;
        this.isRightScore = isRightScore;
        
        // Calculate target setpoint based on scoring side
        double targetSetpoint = isRightScore ? X_SETPOINT : -X_SETPOINT;
        
        // Use your custom APPID controller
        this.strafePID = new APPID(STRAFE_P, STRAFE_I, STRAFE_D, X_TOLERANCE);
        this.strafePID.setMaxOutput(MAX_STRAFE_SPEED);
        this.strafePID.setDesiredValue(targetSetpoint);
        
        // Timers for command management
        this.noTagTimer = new Timer();
        this.stabilityTimer = new Timer();
        
        addRequirements(driveSubsystem);
        
        System.out.println("AutoStrafe created - Right Score: " + isRightScore + ", Target: " + targetSetpoint);
    }

    @Override
    public void initialize() {
        // Reset PID controller
        strafePID.reset();
        
        // Start timers
        noTagTimer.start();
        stabilityTimer.start();
        
        // Lock onto the first tag we see
        if (LimelightHelpers.getTV("limelight-l")) {
            targetTagID = LimelightHelpers.getFiducialID("limelight-l");
            SmartDashboard.putNumber("Target Tag ID", targetTagID);
        }
        
        System.out.println("AutoStrafe initialized - Target Tag ID: " + targetTagID + ", Right Score: " + isRightScore);
    }

    @Override
    public void execute() {
        boolean canSeeTag = LimelightHelpers.getTV("limelight-l");
        double currentTagID = LimelightHelpers.getFiducialID("limelight-l");
        
        // Check if we can see our target tag
        if (!canSeeTag || currentTagID != targetTagID || targetTagID <= 0) {
            // Stop the robot and reset stability timer
            driveSubsystem.drive(0, 0, 0, true);
            stabilityTimer.reset();
            
            SmartDashboard.putString("Strafe Status", "No Target Tag");
            SmartDashboard.putNumber("Current Tag ID", currentTagID);
            return;
        }
        
        // Reset the no-tag timer since we can see the tag
        noTagTimer.reset();
        
        // Get 3D position relative to the tag
        double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-l");
        
        if (positions.length < 6) {
            driveSubsystem.drive(0, 0, 0, true);
            SmartDashboard.putString("Strafe Status", "No Position Data");
            return;
        }
        
        // positions[0] is X position relative to tag (left/right)
        double xPosition = positions[0];
        
        // Calculate strafe output using your custom PID
        double strafeOutput = -strafePID.calcPID(xPosition);
        
        // Send drive command
        driveSubsystem.drive(0, strafeOutput, 0, false);
        
        // Calculate distance from target for finishing logic
        double targetSetpoint = isRightScore ? X_SETPOINT : -X_SETPOINT;
        double distanceFromTarget = Math.abs(xPosition - targetSetpoint);
        
        // Update SmartDashboard
        SmartDashboard.putNumber("X Position", xPosition);
        SmartDashboard.putNumber("Target Setpoint", targetSetpoint);
        SmartDashboard.putNumber("Distance from Target", distanceFromTarget);
        SmartDashboard.putNumber("Strafe Output", strafeOutput);
        SmartDashboard.putString("Strafe Status", "Aligning");
        SmartDashboard.putString("Score Side", isRightScore ? "Right" : "Left");
        
        // Manage stability timer - only reset if we're far from target
        if (distanceFromTarget > FINISH_TOLERANCE) {
            stabilityTimer.reset();
        }
        
        // Debug output
        SmartDashboard.putNumber("Stability Timer", stabilityTimer.get());
        SmartDashboard.putNumber("No Tag Timer", noTagTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        driveSubsystem.drive(0, 0, 0, true);
        
        // Stop timers
        noTagTimer.stop();
        stabilityTimer.stop();
        
        System.out.println("AutoStrafe ended - Interrupted: " + interrupted);
        SmartDashboard.putString("Strafe Status", interrupted ? "Interrupted" : "Finished");
    }

    @Override
    public boolean isFinished() {
        // End if we haven't seen the tag for too long
        if (noTagTimer.hasElapsed(NO_TAG_TIMEOUT)) {
            System.out.println("AutoStrafe finished - No tag timeout");
            return true;
        }
        
        // Check if we can see the tag and get position data
        boolean canSeeTag = LimelightHelpers.getTV("limelight-l");
        double currentTagID = LimelightHelpers.getFiducialID("limelight-l");
        
        if (canSeeTag && currentTagID == targetTagID && targetTagID > 0) {
            double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-l");
            
            if (positions.length >= 6) {
                double xPosition = positions[0];
                double targetSetpoint = isRightScore ? X_SETPOINT : -X_SETPOINT;
                double distanceFromTarget = Math.abs(xPosition - targetSetpoint);
                
                // End if we're close to target and have been stable
                if (distanceFromTarget < FINISH_TOLERANCE && stabilityTimer.hasElapsed(STABILITY_TIME)) {
                    System.out.println("AutoStrafe finished - Within " + FINISH_TOLERANCE + "m of target and stable");
                    return true;
                }
            }
        }
        
        return false;
    }
}