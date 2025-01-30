package frc.robot.subsystems;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    // Thread safety
    private final Lock motorLock = new ReentrantLock();

    public Elevator() {
        // Initialize the motors
        leftMotor = new SparkMax(ElevatorConstants.LEFT_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_ID, MotorType.kBrushless);

        // Validate configuration
        validateConfiguration();

        // Configure the motors
        configureMotor(leftMotor, true);  // Inverted motor
        configureMotor(rightMotor, false); // Non-inverted motor

        // Get encoders and controllers
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();

        // Set up SmartDashboard controls
        setupSmartDashboard();

        // Log initialization
        Logger.recordOutput("Elevator/Initialized", true);
    }

    @SuppressWarnings("unused")
    private void validateConfiguration() {
        if (ElevatorConstants.LEFT_ELEVATOR_ID < 0 || ElevatorConstants.RIGHT_ELEVATOR_ID < 0) {
            throw new IllegalArgumentException("Motor IDs must be non-negative.");
        }
        if (ElevatorConstants.ELEVATOR_kP < 0 || ElevatorConstants.ELEVATOR_kI < 0 || ElevatorConstants.ELEVATOR_kD < 0 || ElevatorConstants.ELEVATOR_kF < 0) {
            throw new IllegalArgumentException("PID constants must be non-negative.");
        }
        if (ElevatorConstants.ELEVATOR_MIN_POSITION >= ElevatorConstants.ELEVATOR_MAX_POSITION) {
            throw new IllegalArgumentException("Elevator min position must be less than max position.");
        }
    }

    private void configureMotor(SparkMax motor, boolean isInverted) {
        SparkMaxConfig config = new SparkMaxConfig();

        // Configure inversion and brake mode
        config.inverted(isInverted)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT); // Set current limit from constants

        // Configure the encoder
        config.encoder.positionConversionFactor(1)
                      .velocityConversionFactor(1);

        // Configure closed loop controller
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .p(ElevatorConstants.ELEVATOR_kP) // Position PID
              .i(ElevatorConstants.ELEVATOR_kI)
              .d(ElevatorConstants.ELEVATOR_kD)
              .velocityFF(ElevatorConstants.ELEVATOR_kF) // Feedforward
              .outputRange(-1, 1)
              .maxMotion.maxVelocity(ElevatorConstants.ELEVATOR_MAX_VELOCITY)
              .maxAcceleration(ElevatorConstants.ELEVATOR_MAX_ACCELERATION)
              .allowedClosedLoopError(ElevatorConstants.ELEVATOR_ALLOWED_ERROR);

        // Apply configuration
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Log motor configuration
        Logger.recordOutput("Elevator/MotorConfigured/" + (isInverted ? "Left" : "Right"), true);
    }

    private void setupSmartDashboard() {
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with diagnostic information
        SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Motor Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Motor Current", rightMotor.getOutputCurrent());

        // Check for encoder reset request
        if (SmartDashboard.getBoolean("Reset Encoder", false)) {
            resetEncoders();
            SmartDashboard.putBoolean("Reset Encoder", false);
        }

        // Safety check for elevator position limits
        if (isElevatorOutOfBounds()) {
            stopElevator();
            Logger.recordOutput("Elevator/OutOfBounds", true);
        }

        // Log diagnostic data
        Logger.recordOutput("Elevator/LeftEncoderPosition", leftEncoder.getPosition());
        Logger.recordOutput("Elevator/RightEncoderPosition", rightEncoder.getPosition());
        Logger.recordOutput("Elevator/LeftMotorCurrent", leftMotor.getOutputCurrent());
        Logger.recordOutput("Elevator/RightMotorCurrent", rightMotor.getOutputCurrent());
    }

    public void setPosition(double targetPosition) {
        // Clamp target position to valid range
        targetPosition = Math.min(Math.max(targetPosition, 
                                ElevatorConstants.ELEVATOR_MIN_POSITION),
                                ElevatorConstants.ELEVATOR_MAX_POSITION);
        
        motorLock.lock();
        try {
            leftController.setReference(targetPosition, ControlType.kPosition);
            rightController.setReference(targetPosition, ControlType.kPosition);
            SmartDashboard.putNumber("Target Position", targetPosition);
            Logger.recordOutput("Elevator/TargetPosition", targetPosition);
        } finally {
            motorLock.unlock();
        }
    }

    public void resetEncoders() {
        motorLock.lock();
        try {
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);
            Logger.recordOutput("Elevator/EncodersReset", true);
        } finally {
            motorLock.unlock();
        }
    }

    public void manualControl(double speed) {
        // Apply deadband and limit speed
        if (Math.abs(speed) < ElevatorConstants.MANUAL_CONTROL_DEADBAND) {
            speed = 0;
        }
        speed = Math.min(Math.max(speed * ElevatorConstants.MANUAL_SPEED_LIMIT, -1), 1);

        motorLock.lock();
        try {
            // Only allow movement if within bounds or moving towards safe range
            if (!isElevatorOutOfBounds() || 
                (getCurrentPosition() <= ElevatorConstants.ELEVATOR_MIN_POSITION && speed > 0) ||
                (getCurrentPosition() >= ElevatorConstants.ELEVATOR_MAX_POSITION && speed < 0)) {
                leftMotor.set(speed);
                rightMotor.set(speed);
            } else {
                stopElevator();
            }
            Logger.recordOutput("Elevator/ManualControlSpeed", speed);
        } finally {
            motorLock.unlock();
        }
    }

    public void stopElevator() {
        motorLock.lock();
        try {
            leftMotor.set(0);
            rightMotor.set(0);
            Logger.recordOutput("Elevator/Stopped", true);
        } finally {
            motorLock.unlock();
        }
    }

    private boolean isElevatorOutOfBounds() {
        double currentPosition = getCurrentPosition();
        return currentPosition < ElevatorConstants.ELEVATOR_MIN_POSITION || 
               currentPosition > ElevatorConstants.ELEVATOR_MAX_POSITION;
    }

    public double getCurrentPosition() {
        motorLock.lock();
        try {
            return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
        } finally {
            motorLock.unlock();
        }
    }

    public boolean isAtTargetPosition() {
        return Math.abs(SmartDashboard.getNumber("Target Position", 0) - getCurrentPosition()) <= 
               ElevatorConstants.ELEVATOR_ALLOWED_ERROR;
    }
}