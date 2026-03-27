package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final int MOTOR_ID = 14;
  public static final int MOTOR_2_ID = 9;
  public static final double GEAR_RATIO = 1.0;

  public static final double kS = 0.13981;
  public static final double kV = 0.12304;
  public static final double kA = 0.016793;
  public static final double kP = 0.0013661;

  public static final double MAX_VELOCITY = 100.0;
  public static final double MAX_ACCEL = 200.0;
  public static final double MAX_JERK = 1000.0;

  public static final double VELOCITY_TOLERANCE_RPM = 50.0;
}
