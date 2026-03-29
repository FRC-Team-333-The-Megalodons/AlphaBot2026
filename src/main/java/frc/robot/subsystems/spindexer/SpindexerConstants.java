package frc.robot.subsystems.spindexer;

public final class SpindexerConstants {
  public static final double GEAR_RATIO = 3.0;
  public static final int MOTOR_ID = 3;

  public static final double SPIN_VOLTAGE_STOPPED = 0.0;
  public static final double SPIN_VOLTAGE = 9.0;
  public static final double SPIN_VOLTAGE_SLOW = 5.0;
  public static final double REVERSE_VOLTAGE = -4.0;

  public static final double MOTOR_SPIN_RPM = 6000.0;
  public static final double SPIN_RPM = MOTOR_SPIN_RPM / GEAR_RATIO;
  public static final double SPIN_RPM_SLOW = 1500.0;
  public static final double EJECT_RPM = -1500.0;
  public static final double VELOCITY_TOLERANCE_RPM = 100.0;

  public static final double JAM_VELOCITY_THRESHOLD_RPS = 15.0;
  public static final double JAM_CURRENT_THRESHOLD_AMPS = 45.0;
  public static final double JAM_DETECT_SECONDS = 0.1;
  public static final double REVERSE_DURATION_SECONDS = 0.15;
  public static final double STARTUP_GRACE_SECONDS = 0.3;

  public static final double kS = 0.32817;
  public static final double kV = 0.097896;
  public static final double kA = 0.0037708;
  public static final double kP = 1.4;

  public static final double MAX_VELOCITY = 9.0;
  public static final double MAX_ACCEL = 25.0;
  public static final double MAX_JERK = 100.0;
}
