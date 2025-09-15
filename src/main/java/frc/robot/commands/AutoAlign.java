// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class AutoAlign extends Command {
  
  private DriveSubsystem driveSubsystem;
  private double tagID = -1;

  public AutoAlign(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    tagID = LimelightHelpers.getFiducialID("");
    System.out.println("SimpleDebugAutoAlign: Starting debug for tag " + tagID);
  }

  @Override
  public void execute() {
    // Check if we can see a valid target
    boolean canSeeTag = LimelightHelpers.getTV("");
    double currentTagID = LimelightHelpers.getFiducialID("");
    
    SmartDashboard.putBoolean("Debug: Can See Tag", canSeeTag);
    SmartDashboard.putNumber("Debug: Current Tag ID", currentTagID);
    SmartDashboard.putNumber("Debug: Target Tag ID", tagID);
    
    // Additional diagnostic info
    SmartDashboard.putNumber("Debug: TX", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("Debug: TY", LimelightHelpers.getTY(""));
    SmartDashboard.putNumber("Debug: TA", LimelightHelpers.getTA(""));
    SmartDashboard.putNumber("Debug: Pipeline", LimelightHelpers.getCurrentPipelineIndex(""));
    SmartDashboard.putString("Debug: Pipeline Type", LimelightHelpers.getCurrentPipelineType(""));
    
    // Check the raw conditions
    boolean tvCondition = LimelightHelpers.getTV("");
    boolean idCondition = (LimelightHelpers.getFiducialID("") == tagID);
    SmartDashboard.putBoolean("Debug: TV Condition", tvCondition);
    SmartDashboard.putBoolean("Debug: ID Condition", idCondition);
    SmartDashboard.putBoolean("Debug: Both Conditions", tvCondition && idCondition);
    
    if (canSeeTag && currentTagID == tagID) {
      // Get the bot pose in target space
      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      
      // Put the three key positions on SmartDashboard
      SmartDashboard.putNumber("Debug: Position[0] (Y)", positions[0]);
      SmartDashboard.putNumber("Debug: Position[2] (X)", positions[2]); 
      SmartDashboard.putNumber("Debug: Position[4] (Rot)", positions[4]);
      
      // Also put the full array length for debugging
      SmartDashboard.putNumber("Debug: Array Length", positions.length);
      
      // Print to console every 50 cycles (~1 second) to avoid spam
      if (System.currentTimeMillis() % 1000 < 20) {
        System.out.println("Positions - X: " + positions[2] + 
                          ", Y: " + positions[0] + 
                          ", Rot: " + positions[4]);
      }
      
      SmartDashboard.putString("Debug: Status", "CAN SEE TARGET - GOOD!");
      
    } else if (!canSeeTag) {
      SmartDashboard.putString("Debug: Status", "TV is false - no valid targets");
    } else if (currentTagID != tagID) {
      SmartDashboard.putString("Debug: Status", "Wrong tag ID: seeing " + currentTagID + " but want " + tagID);
    } else {
      SmartDashboard.putString("Debug: Status", "Unknown issue");
    }
    
    // Keep robot stopped during debug
    driveSubsystem.drive(0, 0, 0, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
    System.out.println("SimpleDebugAutoAlign: Ended");
  }

  @Override
  public boolean isFinished() {
    // Run for 10 seconds then stop
    return false; // Manual stop only - use this to observe values
  }
}