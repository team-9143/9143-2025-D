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
  
  // Constants defining IDs and ports for motors, sensors, and other devices.
  public static class DeviceConstants {}

  public static class AmperConstants {
    // CAN ID for the Spark MAX motor controller.
    public static final int kAmperID = 4;

    // Current limit for the Spark MAX motor controller.
    public static final int kAmperCurrentLimit = 60;

    // Speeds for intaking and scoring. Intaking speed is negative to run the motors in reverse.
    public static final double kAmperIntakeSpeed = 0.4;
    public static final double kAmperScoreSpeed = -0.4;
    public static final double kAmperHoldPositionSpeed = 0.025;
  }

  public static class KitBotConstants {
    // CAN IDs for the Spark MAX motor controllers.
    public static final int kFeederID = 5;
    public static final int kLauncherID = 6;

    // Current limit for the Spark MAX motor controllers.
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for intaking and launching. Launch speeds are negative to run the motors in reverse.
    public static final double kShootLauncherSpeed = 1.5;
    public static final double kShootFeederSpeed = 1.5;
    public static final double kIntakeLauncherSpeed = -1;
    public static final double kIntakeFeederSpeed = -.2;

    // Delay for starting the feeder wheel to allow the launcher wheel to spin up.
    public static final double kFeederDelay = 1;
  }
}
