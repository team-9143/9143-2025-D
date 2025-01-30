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
import frc.robot.Constants.CorAlConstants;

public class CorAl extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final SparkMax intakeMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    // Thread safety
    private final Lock motorLock = new ReentrantLock();

    public CorAl() {
        // Initialize the motors
        pivotMotor = new SparkMax(CorAlConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new SparkMax(CorAlConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Validate configuration
        validateConfiguration();

        // Configure the motors
        configureMotor(pivotMotor, CorAlConstants.PIVOT_MOTOR_INVERTED, true);  // Pivot motor
        configureMotor(intakeMotor, CorAlConstants.INTAKE_MOTOR_INVERTED, false); // Intake motor

        // Get encoders and controllers
        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        // Set up SmartDashboard controls
        setupSmartDashboard();

        // Log initialization
        Logger.recordOutput("CorAl/Initialized", true);
    }

    @SuppressWarnings("unused")
    private void validateConfiguration() {
        if (CorAlConstants.PIVOT_MOTOR_ID < 0 || CorAlConstants.INTAKE_MOTOR_ID < 0) {
            throw new IllegalArgumentException("Motor IDs must be non-negative.");
        }
        if (CorAlConstants.PIVOT_kP < 0 || CorAlConstants.PIVOT_kI < 0 || CorAlConstants.PIVOT_kD < 0 || CorAlConstants.PIVOT_kF < 0) {
            throw new IllegalArgumentException("PID constants must be non-negative.");
        }
        if (CorAlConstants.PIVOT_MIN_ANGLE >= CorAlConstants.PIVOT_MAX_ANGLE) {
            throw new IllegalArgumentException("Pivot min angle must be less than max angle.");
        }
    }

    private void configureMotor(SparkMax motor, boolean isInverted, boolean usePID) {
        SparkMaxConfig config = new SparkMaxConfig();

        // Configure inversion and brake mode
        config.inverted(isInverted)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(usePID ? CorAlConstants.PIVOT_CURRENT_LIMIT : CorAlConstants.INTAKE_CURRENT_LIMIT);

        // Configure the encoder (only for pivot motor)
        if (usePID) {
            config.encoder.positionConversionFactor(1)
                  .velocityConversionFactor(1);
        }

        // Configure closed loop controller (only for pivot motor)
        if (usePID) {
            config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .p(CorAlConstants.PIVOT_kP) // Position PID
                  .i(CorAlConstants.PIVOT_kI)
                  .d(CorAlConstants.PIVOT_kD)
                  .velocityFF(CorAlConstants.PIVOT_kF) // Feedforward
                  .outputRange(-1, 1)
                  .maxMotion.maxVelocity(CorAlConstants.PIVOT_MAX_VELOCITY)
                  .maxAcceleration(CorAlConstants.PIVOT_MAX_ACCELERATION)
                  .allowedClosedLoopError(CorAlConstants.PIVOT_ALLOWED_ERROR);
        }

        // Apply configuration
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Log motor configuration
        Logger.recordOutput("CorAl/MotorConfigured/" + (usePID ? "Pivot" : "Intake"), true);
    }

    private void setupSmartDashboard() {
        SmartDashboard.setDefaultNumber("Pivot Target Angle", 0);
        SmartDashboard.setDefaultBoolean("Reset Pivot Encoder", false);
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with diagnostic information
        SmartDashboard.putNumber("Pivot Encoder Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Motor Current", pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());

        // Check for encoder reset request
        if (SmartDashboard.getBoolean("Reset Pivot Encoder", false)) {
            resetPivotEncoder();
            SmartDashboard.putBoolean("Reset Pivot Encoder", false);
        }

        // Safety check for pivot angle limits
        if (isPivotOutOfBounds()) {
            stopPivot();
            Logger.recordOutput("CorAl/PivotOutOfBounds", true);
        }

        // Log diagnostic data
        Logger.recordOutput("CorAl/PivotEncoderPosition", pivotEncoder.getPosition());
        Logger.recordOutput("CorAl/PivotMotorCurrent", pivotMotor.getOutputCurrent());
        Logger.recordOutput("CorAl/IntakeMotorCurrent", intakeMotor.getOutputCurrent());
    }

    public void setPivotAngle(double targetAngle) {
        // Clamp target angle to valid range
        targetAngle = Math.min(Math.max(targetAngle, 
                                CorAlConstants.PIVOT_MIN_ANGLE),
                                CorAlConstants.PIVOT_MAX_ANGLE);
        
        motorLock.lock();
        try {
            pivotController.setReference(targetAngle, ControlType.kPosition);
            SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
            Logger.recordOutput("CorAl/TargetAngle", targetAngle);
        } finally {
            motorLock.unlock();
        }
    }

    public void resetPivotEncoder() {
        motorLock.lock();
        try {
            pivotEncoder.setPosition(0);
            Logger.recordOutput("CorAl/PivotEncoderReset", true);
        } finally {
            motorLock.unlock();
        }
    }

    public void setIntakeMode(IntakeMode mode) {
        motorLock.lock();
        try {
            switch (mode) {
                case CORAL:
                    intakeMotor.set(CorAlConstants.INTAKE_SPEED);
                    break;
                case ALGAE:
                    intakeMotor.set(CorAlConstants.INTAKE_ALGAE_SPEED);
                    break;
                case STOP:
                default:
                    intakeMotor.set(0);
                    break;
            }
            Logger.recordOutput("CorAl/IntakeMode", mode.name());
        } finally {
            motorLock.unlock();
        }
    }

    public void stopIntake() {
        setIntakeMode(IntakeMode.STOP);
    }

    public void manualPivotControl(double speed) {
        // Apply deadband and limit speed
        if (Math.abs(speed) < CorAlConstants.MANUAL_CONTROL_DEADBAND) {
            speed = 0;
        }
        speed = Math.min(Math.max(speed * CorAlConstants.MANUAL_SPEED_LIMIT, -1), 1);

        motorLock.lock();
        try {
            // Only allow movement if within bounds or moving towards safe range
            if (!isPivotOutOfBounds() || 
                (getPivotAngle() <= CorAlConstants.PIVOT_MIN_ANGLE && speed > 0) ||
                (getPivotAngle() >= CorAlConstants.PIVOT_MAX_ANGLE && speed < 0)) {
                pivotMotor.set(speed);
            } else {
                stopPivot();
            }
            Logger.recordOutput("CorAl/ManualPivotSpeed", speed);
        } finally {
            motorLock.unlock();
        }
    }

    public void stopPivot() {
        motorLock.lock();
        try {
            pivotMotor.set(0);
            Logger.recordOutput("CorAl/PivotStopped", true);
        } finally {
            motorLock.unlock();
        }
    }

    private boolean isPivotOutOfBounds() {
        double currentAngle = getPivotAngle();
        return currentAngle < CorAlConstants.PIVOT_MIN_ANGLE || 
               currentAngle > CorAlConstants.PIVOT_MAX_ANGLE;
    }

    public double getPivotAngle() {
        motorLock.lock();
        try {
            return pivotEncoder.getPosition();
        } finally {
            motorLock.unlock();
        }
    }

    public boolean isAtTargetAngle() {
        return Math.abs(SmartDashboard.getNumber("Pivot Target Angle", 0) - getPivotAngle()) <= 
               CorAlConstants.PIVOT_ALLOWED_ERROR;
    }

    public enum IntakeMode {
        CORAL,
        ALGAE,
        STOP
    }
}