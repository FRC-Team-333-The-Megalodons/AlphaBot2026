package frc.robot.subsystems.transfer;

public final class TransferConstants {
  public static final int MOTOR_ID = 6;
  public static final double GEAR_RATIO = 1.0;

  public static final double TARGET_RPM = 2500.0;
  public static final double VELOCITY_TOLERANCE = 50.0;

  public static final double FEED_VOLTAGE = 11.0;

  public static final double kS = 0.32321;
  public static final double kV = 0.12285;
  public static final double kA = 0.0027834;
  public static final double kP = 0.031391;

  public static final double MAX_VELOCITY = 100;
  public static final double MAX_ACCEL = 200.0;
  public static final double MAX_JERK = 1000.0;

  //Flywheel-relative feed tuning

  /** Proportional mode: transferRPM = flywheelRPM × this ratio. Start at 1.1 (10% faster). */
  public static final double PROPORTIONAL_FEED_RATIO = 1.1;

  /** Additive mode: transferRPM = flywheelRPM + this offset. Start at +700 RPM. */
  public static final double ADDITIVE_FEED_OFFSET_RPM = 700.0;

  public static final double MIN_FEED_RPM = 1000.0;
}
