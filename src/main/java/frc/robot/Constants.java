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
    
    // Motor configurations
    public static final boolean LEFT_MOTOR_INVERTED = true;
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final com.ctre.phoenix.motorcontrol.NeutralMode NEUTRAL_MODE =
            com.ctre.phoenix.motorcontrol.NeutralMode.Brake;

    // Limit switch configurations
    public static final boolean FORWARD_SOFT_LIMIT_ENABLE = true;
    public static final int FORWARD_SOFT_LIMIT = 20000; // Example value
    public static final boolean REVERSE_SOFT_LIMIT_ENABLE = true;
    public static final int REVERSE_SOFT_LIMIT = 0; // Example value
  }
}