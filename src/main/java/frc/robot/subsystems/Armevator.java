// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.utils.APPID;

public class Armevator extends SubsystemBase {

  public enum ArmevatorState {
    DEFAULT,
    INTAKE,
    LEVEL_4,
    LEVEL_3,
    LEVEL_2,
    SCORING
  }

  /** Creates a new Armevator. */
  private final SparkMax armMotor = new SparkMax(55, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();

  private final SparkFlex eleMotor1 = new SparkFlex(50, MotorType.kBrushless);
  private final SparkFlex eleMotor2 = new SparkFlex(51, MotorType.kBrushless);

  private final RelativeEncoder eleEncoder1 = eleMotor1.getEncoder();

  private final APPID armPID;
  private final APPID elePID;

  // Setup Variables
  private static final double ARM_GEAR_RATIO = 70;
  private static final double ENCODER_TO_DEGREES = 360/ARM_GEAR_RATIO;
  private static final double ELE_GEAR_RATIO = 3;
  private double targetElePos = 0;
  private double targetArmPos = 0;

  public Armevator() {

    armMotor.configure(Configs.ArmSubsystem.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armEncoder.setPosition(0);

    eleMotor1.configure(Configs.ElevatorSubystem.elevatorConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    eleMotor2.configure(Configs.ElevatorSubystem.elevatorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armPID = new APPID(0.025, 0, 0, 1);
    armPID.setMaxOutput(0.7);

    elePID = new APPID(0.07, 0, 0, 1);
    elePID.setDesiredValue(0.5);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runArmPID();
    runElePID();
  }

  // Elevator ---------
  public void setEleTarget(double position) {
    targetElePos = position;
    elePID.setDesiredValue(targetElePos);
  }

  public void runElePID() {
    double currentElePos = eleEncoder1.getPosition();
    double output = elePID.calcPID(currentElePos);
    eleMotor1.set(output);
    eleMotor2.set(output);
  }

  public boolean eleAtTarget() {
    return elePID.isDone();
  }

  public void stopElevator() {
    eleMotor1.stopMotor();
    eleMotor2.stopMotor();
  }

  public double getElevatorPosition() {
    return eleEncoder1.getPosition();
  }

  // ARM ---------
  public void setArmTarget(double angle) {
    targetArmPos = angle;
    armPID.setDesiredValue(targetArmPos);
  }

  public void runArmPID() {
    double currentPos = armEncoder.getPosition() * ENCODER_TO_DEGREES;
    double output = armPID.calcPID(currentPos);
    armMotor.set(output);
  }

  public boolean armAtTarget() {
    return armPID.isDone();
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public double getArmPosition() {
    return armEncoder.getPosition() * ENCODER_TO_DEGREES;
  }

  // State Checks --------

  public boolean isAtDefault() {
    // Check if both arm and elevator are at default positions
    return Math.abs(getArmPosition() - 0) < 2 && Math.abs(getElevatorPosition() - 10) < 2;
    // Adjust tolerance values (2) as needed for your system
  }

  public boolean isAtLevelPosition() {
      // Check if robot is at any level position (for scoring validation)
      return isAtLevel2() || isAtLevel3() || isAtLevel4();
  }

  public boolean isAtLevel2() {
      return Math.abs(getArmPosition() - 50) < 2 && Math.abs(getElevatorPosition() - 0) < 2;
  }

  public boolean isAtLevel3() {
      return Math.abs(getArmPosition() - 190) < 2 && Math.abs(getElevatorPosition() - 0) < 2;
  }

  public boolean isAtLevel4() {
      return Math.abs(getArmPosition() - 190) < 2 && Math.abs(getElevatorPosition() - 23) < 2;
  }

}
