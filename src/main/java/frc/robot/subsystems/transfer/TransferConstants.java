package frc.robot.subsystems.transfer;

public final class TransferConstants {
  public static final int MOTOR_ID = 6;
  public static final double GEAR_RATIO = 1.0;

  public static final double TARGET_RPM = -2000.0;
  public static final double VELOCITY_TOLERANCE = 50.0;

  public static final double FEED_VOLTAGE = -11.0;

  // Starting Gains
  public static final double kS = 0.2;
  public static final double kV = 0.12;
  public static final double kA = 0.01;
  public static final double kP = 0.1;

  public static final double MAX_ACCEL = 200.0;
  public static final double MAX_JERK = 1000.0;
}
