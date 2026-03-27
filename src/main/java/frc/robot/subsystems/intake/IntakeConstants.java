package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int MOTOR_ID = 5;
  public static final double GEAR_RATIO = 1.0;

  public static final double INTAKE_VOLTS = 7.2;
  public static final double EJECT_VOLTS = -7.2;

  public static final double INTAKE_RPM = 3000.0;
  public static final double EJECT_RPM = -3000.0;
  public static final double VELOCITY_TOLERANCE_RPM = 100.0;

  public static final double kS = 0.27809;
  public static final double kV = 0.12369;
  public static final double kA = 0.0037911;
  public static final double kP = 0.0028356;

  public static final double MAX_VELOCITY = 100.0;
  public static final double MAX_ACCEL = 200.0;
  public static final double MAX_JERK = 1000.0;
}
