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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CorAlConstants;

public class CorAl extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final SparkMax intakeMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;
    private final ShuffleboardTab tab;
    private final SimpleWidget pivotTargetAngleWidget;
    private final SimpleWidget resetPivotEncoderWidget;

    public CorAl() {
        // Initialize motors
        pivotMotor = new SparkMax(CorAlConstants.CORAL_PIVOT_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new SparkMax(CorAlConstants.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Configure motors
        configureMotor(pivotMotor, CorAlConstants.CORAL_PIVOT_MOTOR_INVERTED, true);  // Pivot motor
        configureMotor(intakeMotor, CorAlConstants.CORAL_INTAKE_MOTOR_INVERTED, false); // Intake motor

        // Get encoders and controllers
        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        // Initialize Shuffleboard tab and widgets
        tab = Shuffleboard.getTab("CorAl");
        pivotTargetAngleWidget = tab.add("Pivot Target Angle", 0)
                                    .withPosition(0, 0)
                                    .withSize(2, 1);
        resetPivotEncoderWidget = tab.add("Reset Pivot Encoder", false)
                                     .withPosition(2, 0)
                                     .withSize(2, 1);

        // Configure Shuffleboard
        configureShuffleboard();
    }

    private void configureMotor(SparkMax motor, boolean isInverted, boolean usePID) {
        SparkMaxConfig config = new SparkMaxConfig();

        // Configure inversion and brake mode
        config.inverted(isInverted)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(usePID ? CorAlConstants.CORAL_PIVOT_CURRENT_LIMIT : CorAlConstants.CORAL_INTAKE_CURRENT_LIMIT);

        // Configure the encoder (only for pivot motor)
        if (usePID) {
            config.encoder.positionConversionFactor(CorAlConstants.CORAL_PIVOT_POSITION_CONVERSION)
                  .velocityConversionFactor(CorAlConstants.CORAL_PIVOT_VELOCITY_CONVERSION);
        }

        // Configure closed loop controller (only for pivot motor)
        if (usePID) {
            config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .p(CorAlConstants.CORAL_PIVOT_kP) // Position PID
                  .i(CorAlConstants.CORAL_PIVOT_kI)
                  .d(CorAlConstants.CORAL_PIVOT_kD)
                  .outputRange(-1, 1);
        }

        // Apply configuration
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configureShuffleboard() {
        // Status indicators
        tab.addNumber("Pivot Angle", this::getPivotAngle)
           .withPosition(0, 1)
           .withSize(2, 1);
        
        tab.addNumber("Pivot Current", pivotMotor::getOutputCurrent)
           .withPosition(2, 1)
           .withSize(2, 1);
           
        tab.addNumber("Intake Current", intakeMotor::getOutputCurrent)
           .withPosition(4, 1)
           .withSize(2, 1);

        // Status booleans
        tab.addBoolean("Pivot Out of Bounds", this::isPivotOutOfBounds)
           .withPosition(0, 2)
           .withSize(2, 1);
           
        tab.addBoolean("At Target Angle", this::isAtTargetAngle)
           .withPosition(2, 2)
           .withSize(2, 1);
    }

    @Override
    public void periodic() {
        // Check for encoder reset
        if (resetPivotEncoderWidget.getEntry().getBoolean(false)) {
            resetPivotEncoder();
            resetPivotEncoderWidget.getEntry().setBoolean(false);
        }

        // Safety check
        if (isPivotOutOfBounds()) {
            stopPivot();
        }
    }

    public void setPivotAngle(double targetAngle) {
        // Clamp target angle
        targetAngle = Math.min(Math.max(targetAngle, 
                              CorAlConstants.CORAL_PIVOT_MIN_ANGLE),
                              CorAlConstants.CORAL_PIVOT_MAX_ANGLE);
        
        pivotController.setReference(targetAngle, ControlType.kPosition);
        pivotTargetAngleWidget.getEntry().setDouble(targetAngle);
    }

    public void resetPivotEncoder() {
        pivotEncoder.setPosition(0);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        setIntakeSpeed(0);
    }

    public void manualPivotControl(double speed) {
        // Apply deadband and limits
        if (Math.abs(speed) < CorAlConstants.CORAL_MANUAL_CONTROL_DEADBAND) {
            speed = 0;
        }
        speed = Math.min(Math.max(speed * CorAlConstants.CORAL_MANUAL_SPEED_LIMIT, -1), 1);

        // Safety checks
        if (!isPivotOutOfBounds() || 
            (getPivotAngle() <= CorAlConstants.CORAL_PIVOT_MIN_ANGLE && speed > 0) ||
            (getPivotAngle() >= CorAlConstants.CORAL_PIVOT_MAX_ANGLE && speed < 0)) {
            pivotMotor.set(speed);
        } else {
            stopPivot();
        }
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    private boolean isPivotOutOfBounds() {
        double currentAngle = getPivotAngle();
        return currentAngle < CorAlConstants.CORAL_PIVOT_MIN_ANGLE || 
               currentAngle > CorAlConstants.CORAL_PIVOT_MAX_ANGLE;
    }

    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    public boolean isAtTargetAngle() {
        return Math.abs(pivotTargetAngleWidget.getEntry().getDouble(0) - getPivotAngle()) <= 
               CorAlConstants.CORAL_PIVOT_ALLOWED_ERROR;
    }
}