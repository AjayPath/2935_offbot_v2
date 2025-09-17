// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Armevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Pose;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Armevator m_armevator = new Armevator();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Go to Default (can always do this)
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> m_armevator.manualReturnToDefault()), 
                
                new PrintCommand("WARNING: Sequence active - forcing return to default!")
                    .andThen(new InstantCommand(() -> m_armevator.manualReturnToDefault())), 
                    
                () -> m_armevator.canReturnToDefault())
    );

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .onTrue(
            new ConditionalCommand(
                // If safe for intake, execute
                new InstantCommand(() -> m_armevator.setEleTarget(0)),
                // If not safe, give specific error message
                new PrintCommand("INTAKE BLOCKED: - Press Right Bumper to return to default first!"),
                // Condition: check if safe for intake
                () -> m_armevator.isSafeForIntakeCommand()
            )
        );

    // Level 2 - only if at default
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .onTrue(
            new ConditionalCommand(
            // If at default, do Level 2 sequence
            new InstantCommand(() -> m_armevator.setArmTarget(50))
                .andThen(new WaitUntilCommand(() -> m_armevator.armAtTarget()))
                .andThen(new InstantCommand(() -> m_armevator.setEleTarget(0))),
            // Otherwise, print error or go to default first
            new PrintCommand("Must be at default position before going to Level 2!"),
            // Condition: check if at default
            () -> m_armevator.isSafeForLevelCommands()
        )
    );

    // Level 3 - only if at default  
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
    .onTrue(
        new ConditionalCommand(
            // If at default, do Level 3 sequence
            new InstantCommand(() -> m_armevator.setArmTarget(190))
                .andThen(new WaitUntilCommand(() -> m_armevator.armAtTarget()))
                .andThen(new InstantCommand(() -> m_armevator.setEleTarget(0))),
            // Otherwise, print error
            new PrintCommand("Must be at default position before going to Level 3!"),
            // Condition: check if at default
            () -> m_armevator.isSafeForLevelCommands()
        )
    );

    // Level 4 - only if at default
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .onTrue(
            new ConditionalCommand(
            // If at default, do Level 4 sequence
            new InstantCommand(() -> m_armevator.setArmTarget(190))
                .andThen(new WaitUntilCommand(() -> m_armevator.armAtTarget()))
                .andThen(new InstantCommand(() -> m_armevator.setEleTarget(23.5))),
            // Otherwise, print error
            new PrintCommand("Must be at default position before going to Level 4!"),
            // Condition: check if at default
            () -> m_armevator.isSafeForLevelCommands()
        )
    );
    
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .onTrue(
            new ConditionalCommand(
            // If at level position, go to scoring
            new InstantCommand(() -> m_armevator.setArmTarget(100)),
            // Otherwise, print error
            new PrintCommand("Must be at Level 2, 3, or 4 before scoring!"),
            // Condition: check if at any level position
            //() -> m_armevator.isAtLevelPosition() && !m_armevator.isAnySequenceActive()
            () -> m_armevator.isSafeForScoringCommand()
        )
    );
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  public void logDriveStatus() {
    DriveSubsystem drive = getDriveSubsystem();

    // Get gyro
    double heading = drive.getHeading();

    // Get odometry
    Pose2d wpilibPose = drive.getPose();
    frc.robot.utils.Pose customPose = drive.getCustomPose();

    // Log to SmartDashboard
    SmartDashboard.putNumber("RC Gyro deg", heading);
    SmartDashboard.putNumber("RC WPILib X", wpilibPose.getX());
    SmartDashboard.putNumber("RC WPILib Y", wpilibPose.getY());
    SmartDashboard.putNumber("RC WPILib Angle", wpilibPose.getRotation().getDegrees());

    SmartDashboard.putNumber("RC APOdometry X", customPose.GetXValue());
    SmartDashboard.putNumber("RC APOdometry Y", customPose.GetYValue());
    SmartDashboard.putNumber("RC APOdometry Angle", customPose.GetAngleValue());
  }

}