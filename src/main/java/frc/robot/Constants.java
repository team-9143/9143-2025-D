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

    public static final int LEFT_ELEVATOR_ID = 8;
    public static final int RIGHT_ELEVATOR_ID = 9;

    // PID values for position control
    public static final double ELEVATOR_kP = 0.4;
    public static final double ELEVATOR_kI = 0.0;
    public static final double ELEVATOR_kD = 0.0;

    // MAXMotion parameters
    public static final double ELEVATOR_MAX_VELOCITY = 1000.0; // Encoder units per second
    public static final double ELEVATOR_MAX_ACCELERATION = 1000.0; // Encoder units per second squared
    public static final double ELEVATOR_ALLOWED_ERROR = 1.0; // Tolerance in encoder units
  }
}