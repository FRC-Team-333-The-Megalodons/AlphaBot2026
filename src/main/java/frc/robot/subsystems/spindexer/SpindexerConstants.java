package frc.robot.subsystems.spindexer;

public final class SpindexerConstants {
  public static final double GEAR_RATIO = 1.0;
  public static final int MOTOR_ID = 3;
  public static final double MOTOR_VOLTS = 0.0;

  public static final double TARGET_RPM = 3000.0;
  public static final double VELOCITY_TOLERANCE = 50.0;

  // Starting Gains
  public static final double kS = 0.2;
  public static final double kV = 0.12;
  public static final double kA = 0.01;
  public static final double kP = 0.1;

  public static final double MAX_ACCEL = 150.0;
  public static final double MAX_JERK = 800.0;
}
