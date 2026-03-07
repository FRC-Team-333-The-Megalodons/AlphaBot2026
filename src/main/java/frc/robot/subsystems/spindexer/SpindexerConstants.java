package frc.robot.subsystems.spindexer;

public final class SpindexerConstants {
  public static final double GEAR_RATIO = 1.0;
  public static final int MOTOR_ID = 3;
  public static final double MOTOR_VOLTS = 0.0;

  public static final double TARGET_RPM = 3000.0;
  public static final double VELOCITY_TOLERANCE = 50.0;

  // Normal spin voltage (forward)
  public static final double SPIN_VOLTAGE = 6.0;

  public static final double JAM_VELOCITY_THRESHOLD_RPS = 15.0;

  
  public static final double JAM_CURRENT_THRESHOLD_AMPS = 45.0;

 
  public static final double JAM_DETECT_SECONDS = 0.1;

  
  public static final double REVERSE_VOLTAGE = -4.0;

  public static final double REVERSE_DURATION_SECONDS = 0.15;

 
  public static final double STARTUP_GRACE_SECONDS = 0.3;

 
  public static final double kV = 0.12;
  public static final double kA = 0.01;
  public static final double kP = 0.1;

  public static final double MAX_ACCEL = 150.0;
  public static final double MAX_JERK = 800.0;
}
