package frc.robot;

/*
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Constants related to operator input devices like controllers and button mappings.
  public static class OperatorConstants {}
  
  // Constants defining IDs and ports for motors, sensors, and other devices.
  public static class DeviceConstants {}

  // Constants specifying physical dimensions and characteristics of the robot.
  public static class PhysicalConstants {}

  // Constants used for autonomous routines, including speeds and trajectory parameters.
  public static class AutoConstants {}

  // Constants for PID controller tuning values for various subsystems.
  public static class PIDConstants {}

  // Constants for vision processing, including camera parameters and pipeline indices.
  public static class VisionConstants {}

  // Constants for motor configurations, such as gear ratios and current limits.
  public static class MotorConstants {}

  // Constants for the elevator subsystem (built off of FRC 3255)
  public static class ElevatorConstants {

    public static final int LEFT_ELEVATOR_ID = 58;
    public static final int RIGHT_ELEVATOR_ID = 59;

    // PID Constants
    public static final double ELEVATOR_kP = 0.4; // Proportional gain
    public static final double ELEVATOR_kI = 0.0; // Integral gain
    public static final double ELEVATOR_kD = 0.0; // Derivative gain
    public static final double ELEVATOR_kF = 0.0; // Feedforward gain

    // Motion Profiling Constants
    public static final double ELEVATOR_MAX_VELOCITY = 1000; // Max velocity in units per second
    public static final double ELEVATOR_MAX_ACCELERATION = 500; // Max acceleration in units per second squared
    public static final double ELEVATOR_ALLOWED_ERROR = 0.5; // Allowed closed-loop error in units
    public static final double ELEVATOR_MIN_VELOCITY = 0; // Minimum velocity for Smart Motion

    // Motor Inversion
    public static final boolean LEFT_MOTOR_INVERTED = true; // Invert if necessary
    public static final boolean RIGHT_MOTOR_INVERTED = false; // Invert if necessary

    // Current Limits
    public static final int ELEVATOR_CURRENT_LIMIT = 60; // Set current limit in amps

    // Elevator Position Limits
    public static final double ELEVATOR_MIN_POSITION = 0; // Minimum allowed position (e.g., fully retracted)
    public static final double ELEVATOR_MAX_POSITION = 100; // Maximum allowed position (e.g., fully extended)

    // Manual Control Constants
    public static final double MANUAL_CONTROL_DEADBAND = 0.1; // Deadband for manual control input
    public static final double MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual control (0 to 1)
  }

  public final class CorAlConstants {
    // Motor IDs
    public static final int PIVOT_MOTOR_ID = 60; // Replace with actual CAN ID
    public static final int INTAKE_MOTOR_ID = 61; // Replace with actual CAN ID

    // Motor Inversion
    public static final boolean PIVOT_MOTOR_INVERTED = false; // Invert if necessary
    public static final boolean INTAKE_MOTOR_INVERTED = false; // Invert if necessary

    // Current Limits
    public static final int PIVOT_CURRENT_LIMIT = 40; // Set current limit in amps
    public static final int INTAKE_CURRENT_LIMIT = 30; // Set current limit in amps

    // PID Constants for Pivot
    public static final double PIVOT_kP = 0.1; // Proportional gain
    public static final double PIVOT_kI = 0.0; // Integral gain
    public static final double PIVOT_kD = 0.0; // Derivative gain
    public static final double PIVOT_kF = 0.0; // Feedforward gain

    // Motion Profiling Constants for Pivot
    public static final double PIVOT_MAX_VELOCITY = 1000; // Max velocity in units per second
    public static final double PIVOT_MAX_ACCELERATION = 500; // Max acceleration in units per second squared
    public static final double PIVOT_ALLOWED_ERROR = 0.5; // Allowed closed-loop error in units
    public static final double PIVOT_MIN_VELOCITY = 0; // Minimum velocity for Smart Motion

    // Pivot Position Limits
    public static final double PIVOT_MIN_ANGLE = 0; // Minimum allowed angle (e.g., fully retracted)
    public static final double PIVOT_MAX_ANGLE = 90; // Maximum allowed angle (e.g., fully extended)

    // Intake Constants
    public static final double INTAKE_SPEED = 0.8; // Speed for intaking/outtaking
    public static final double INTAKE_ALGAE_SPEED = -0.8; // Speed for intaking algae (opposite direction)

    // Manual Control Constants
    public static final double MANUAL_CONTROL_DEADBAND = 0.1; // Deadband for manual control input
    public static final double MANUAL_SPEED_LIMIT = 0.5; // Speed limit for manual control (0 to 1)
  }
}
