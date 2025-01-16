// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends TimedRobot implements Subsystem {

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private SparkClosedLoopController leftController;
    private SparkClosedLoopController rightController;
    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;

    public Elevator() {

        // Initialize the motors
        leftMotor = new SparkMax(ElevatorConstants.LEFT_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_ID, MotorType.kBrushless);

        // Configure the motors
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        configureMotor(leftMotor, leftConfig, true);  // Inverted motor
        configureMotor(rightMotor, rightConfig, false); // Non-inverted motor

        // Get encoders and controllers
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();

        // Set up SmartDashboard controls
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultBoolean("Manual Control", true);
        SmartDashboard.setDefaultNumber("Manual Speed", 0);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    private void configureMotor(SparkMax motor, SparkMaxConfig config, boolean isInverted) {

        // Configure inversion and brake mode
        config.inverted(isInverted)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(60); // Set current limit to 60 amps

        // Configure the encoder
        config.encoder.positionConversionFactor(1)
                      .velocityConversionFactor(1);

        // Configure closed loop controller
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .p(ElevatorConstants.ELEVATOR_kP) // Position PID
              .i(ElevatorConstants.ELEVATOR_kI)
              .d(ElevatorConstants.ELEVATOR_kD)
              .outputRange(-1, 1)
              .maxMotion.maxVelocity(ElevatorConstants.ELEVATOR_MAX_VELOCITY)
              .maxAcceleration(ElevatorConstants.ELEVATOR_MAX_ACCELERATION)
              .allowedClosedLoopError(ElevatorConstants.ELEVATOR_ALLOWED_ERROR);

        // Apply configuration
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void teleopPeriodic() {

        if (SmartDashboard.getBoolean("Reset Encoder", false)) {
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
            SmartDashboard.putBoolean("Reset Encoder", false);
        }

        /*
        boolean manualControl = SmartDashboard.getBoolean("Manual Control", true);

        if (manualControl) {
            // Manual control
            double manualSpeed = SmartDashboard.getNumber("Manual Speed", 0);
            leftMotor.set(manualSpeed);
            rightMotor.set(manualSpeed);
        } else {
            // Position control
            double targetPosition = SmartDashboard.getNumber("Target Position", 0);
            leftController.setReference(targetPosition, ControlType.kPosition);
            rightController.setReference(targetPosition, ControlType.kPosition);
        }
        */

        // Display encoder positions on the dashboard
        SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());
    }

    public void setPosition(double targetPosition) {
        leftController.setReference(targetPosition, ControlType.kPosition);
        rightController.setReference(targetPosition, ControlType.kPosition);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void manualControl(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    @Override
    public void robotPeriodic() {
        // Update encoder values for monitoring
        SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());
    }
}