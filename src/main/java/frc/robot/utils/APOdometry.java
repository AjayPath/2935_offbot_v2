// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;

/** Add your docs here. */
public class APOdometry {

    private static APOdometry instance;
    private final Pigeon2 gyro;
    private final List<MAXSwerveModule> swerveMods;
    private final List<Pose> modulePoses;
    private final List<Vector> wheelOffsets;
    private final double[] lastWheelDistances;
    private Pose lastCenter;
    private double lastTime;
    private Vector lastVelocity;

    // Constructor
    private APOdometry(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
        lastWheelDistances = new double[swerveMods.size()];
        this.swerveMods = swerveMods;
        this.modulePoses = new ArrayList<>(swerveMods.size());
        this.gyro = gyro;

        double a = DriveConstants.kWheelBase / 2;
        double b = DriveConstants.kTrackWidth / 2;

        wheelOffsets = List.of(
            new Vector(a, b),
            new Vector(a, -b),
            new Vector(-a, b),
            new Vector(-a, -b)
        );

        // Set up all modules at (0,0) with current wheel angle
        for (int i = 0; i < swerveMods.size(); i++) {
            double initialAngle = swerveMods.get(i).getPosition().angle.getDegrees();
            modulePoses.add(new Pose(0, 0, initialAngle));
        }

        // Store continuous angle internally
        lastCenter = new Pose(0, 0, gyro.getRotation2d().getDegrees());
        lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    public static APOdometry getInstance(List<MAXSwerveModule> swerveMods, Pigeon2 gyro) {
        if (instance == null) {
            instance = new APOdometry(swerveMods, gyro);
        }
        return instance;
    }

    // Update Values
    public void update() {
        for (int i = 0; i < swerveMods.size(); i++) {

            // Read mods reported pos and angle
            double currentDistance = swerveMods.get(i).getPosition().distanceMeters;
            double deltaDistance = currentDistance - lastWheelDistances[i];
            double wheelAngle = swerveMods.get(i).getPosition().angle.getDegrees();
            double wheelAngleRad = Math.toRadians(wheelAngle);
            lastWheelDistances[i] = currentDistance;
            
            double dx = Math.cos(wheelAngleRad) * deltaDistance;
            double dy = Math.sin(wheelAngleRad) * deltaDistance;
            
            // Update the Pose for the wheel
            modulePoses.get(i).SetX(modulePoses.get(i).GetXValue() + dx);
            modulePoses.get(i).SetY(modulePoses.get(i).GetYValue() + dy);
            modulePoses.get(i).SetAngle(wheelAngle);
        }

        Pose currentCenter = calculateCenter();
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        double dt = currentTime - lastTime;
        if (dt > 0) {
            double dx = currentCenter.GetXValue() - lastCenter.GetXValue();
            double dy = currentCenter.GetYValue() - lastCenter.GetYValue();
    
            double mag = Math.sqrt(dx * dx + dy * dy) / dt;
            Rotation2d angle = new Rotation2d(Math.atan2(dy, dx));

            lastVelocity = new Vector(mag, angle);
        }

        lastCenter = currentCenter;
        lastTime = currentTime;
    }

    public List<Pose> getModulePoses() {
        return new ArrayList<>(modulePoses);
    }

    public Pose getModulePose (int index) {
        return modulePoses.get(index);
    }

    public void logWheelPoses() {
        for (int i = 0; i < swerveMods.size(); i++) {
            Pose p = modulePoses.get(i);
            SmartDashboard.putNumber("Wheel " + i + " X", p.GetXValue());
            SmartDashboard.putNumber("Wheel " + i + " Y", p.GetYValue());
            SmartDashboard.putNumber("Wheel " + i + " Angle", p.GetAngleValue());
        }
    }

    public Pose getCenterFromWheel (Pose wheelPose, Vector wheelOffset) {

        double yaw = gyro.getRotation2d().getDegrees();
        double offsetAngleRad = Math.toRadians(yaw + wheelOffset.GetAngle().getDegrees());
        double offsetX = wheelOffset.GetMag() * Math.cos(offsetAngleRad);
        double offsetY = wheelOffset.GetMag() * Math.sin(offsetAngleRad);

        Pose centerPose = new Pose(
            wheelPose.GetXValue() - offsetX,
            wheelPose.GetYValue() - offsetY,
            yaw
        );

        return centerPose;
    }

    public Pose calculateCenter() {

        double sumX = 0;
        double sumY = 0;

        for (int i = 0; i < modulePoses.size(); i++) {
            Pose centerFromThisWheel = getCenterFromWheel(modulePoses.get(i), wheelOffsets.get(i));
            sumX += centerFromThisWheel.GetXValue();
            sumY += centerFromThisWheel.GetYValue();
        }

        double avgX = sumX / modulePoses.size();
        double avgY = sumY / modulePoses.size();
        // Store continuous angle from gyro
        double avgAngle = gyro.getRotation2d().getDegrees();
        
        return new Pose(avgX, avgY, avgAngle);
    }

    public void setPose(Pose newPose) {
        
        for (int i = 0; i < swerveMods.size(); i++) {

            Vector offset = wheelOffsets.get(i);
            double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

            modulePoses.get(i).SetPose(
                offset.GetXValue() + newPose.GetXValue(),
                offset.GetYValue() + newPose.GetYValue(),
                steerAngle
            );

            lastCenter = newPose;

            lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
        }

        calculateCenter();
    }

    public void reset() {
        for (int i = 0; i < swerveMods.size(); i++) {
            Vector offset = wheelOffsets.get(i);
            double steerAngle = swerveMods.get(i).getPosition().angle.getDegrees();

            modulePoses.get(i).SetPose(
                offset.GetXValue(),
                offset.GetYValue(),
                steerAngle
            );

            lastWheelDistances[i] = swerveMods.get(i).getPosition().distanceMeters;
        }
        lastCenter = new Pose(0, 0, gyro.getRotation2d().getDegrees());
    }

    /**
     * Get the current pose with continuous angle (for internal calculations)
     * @return Pose with continuous angle
     */
    public Pose getPoseContinuous() {
        return lastCenter;
    }

    /**
     * Get the current pose with normalized angle (0-360)
     * @return Pose with angle normalized to [0, 360) range
     */
    public Pose getPose() {
        Pose normalizedPose = new Pose(lastCenter);
        normalizedPose.SetAngle(Calculations.NormalizeAngle360(lastCenter.GetAngleValue()));
        return normalizedPose;
    }

    public Vector getVelocity() {
        return lastVelocity;
    }

    public void logCenterPose() {
        Pose center = getPose(); // This will be normalized
        Vector robotVel = getVelocity();
        SmartDashboard.putNumber("Center X", center.GetXValue());
        SmartDashboard.putNumber("Center Y", center.GetYValue());
        SmartDashboard.putNumber("Center Angle", center.GetAngleValue());
        SmartDashboard.putNumber("Center Angle Continuous", lastCenter.GetAngleValue()); // Show both

        SmartDashboard.putNumber("Speed", robotVel.GetMag());
        SmartDashboard.putNumber("Direction", robotVel.GetAngle().getDegrees());
    }
}