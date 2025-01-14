// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    final WPI_TalonSRX leftMotorFollower;
    final WPI_TalonSRX rightMotorLeader;

  /** Creates a new Elevator. */
  public Elevator() {
    // Initialize motors
    leftMotorFollower = new WPI_TalonSRX(ElevatorConstants.LEFT_ELEVATOR_ID);
    rightMotorLeader = new WPI_TalonSRX(ElevatorConstants.RIGHT_ELEVATOR_ID);

    // Reset to factory defaults
    leftMotorFollower.configFactoryDefault();
    rightMotorLeader.configFactoryDefault();

    // Configure follower
    leftMotorFollower.follow(rightMotorLeader, FollowerType.PercentOutput);

    // Configure neutral mode (brake)
    leftMotorFollower.setNeutralMode(ElevatorConstants.NEUTRAL_MODE);
    rightMotorLeader.setNeutralMode(ElevatorConstants.NEUTRAL_MODE);

    // Configure inverted behavior
    leftMotorFollower.setInverted(ElevatorConstants.LEFT_MOTOR_INVERTED);
    rightMotorLeader.setInverted(ElevatorConstants.RIGHT_MOTOR_INVERTED);

    // Configure limit switches
    rightMotorLeader.configForwardSoftLimitEnable(ElevatorConstants.FORWARD_SOFT_LIMIT_ENABLE);
    rightMotorLeader.configForwardSoftLimitThreshold(ElevatorConstants.FORWARD_SOFT_LIMIT);
    rightMotorLeader.configReverseSoftLimitEnable(ElevatorConstants.REVERSE_SOFT_LIMIT_ENABLE);
    rightMotorLeader.configReverseSoftLimitThreshold(ElevatorConstants.REVERSE_SOFT_LIMIT);
  }

  public void setPosition(double setpoint) {
    // Use Motion Magic or Position control for precise positioning
    rightMotorLeader.set(ControlMode.Position, setpoint);
  }

  public void resetSensorPosition() {
    rightMotorLeader.setSelectedSensorPosition(0);
    leftMotorFollower.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // Output diagnostics to SmartDashboard
    SmartDashboard.putNumber("Elevator/Right/Position", rightMotorLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator/Left/Position", leftMotorFollower.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator/Right/Current", rightMotorLeader.getStatorCurrent());
    SmartDashboard.putNumber("Elevator/Left/Current", leftMotorFollower.getStatorCurrent());
  }
}