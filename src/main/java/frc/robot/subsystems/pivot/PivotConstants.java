package frc.robot.subsystems.pivot;

public final class PivotConstants {

  public static final int MOTOR_ID = 4;

  // TODO: set proper gear ratio(Vlad knowns)
  public static final double GEAR_RATIO = 13.75;

  // PID
  // public static final double kP = 15.0;
  public static final double kP = 8.5;
  public static final double kI = 0.0;
  public static final double kD = 0.5;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.0;

  // TODO: Tune these.
  public static final double kCruiseVelocity = 0.5;
  public static final double kAcceleration = 1;
  public static final double kJerk = 4;

  // Soft limits
  // TODO: set  min/max based on physical range of motion
  public static final double kMinAngleDeg = -2.0;
  public static final double kMaxAngleDeg = 167.0;

  // Named positions
  // Named positions
  public static final double kUpAngleDeg = 0.0; // Physical hard stop at the top
  public static final double kDownAngleDeg = 165.0;

  // Tolerances
  public static final double POSITION_TOLERANCE_DEG = 1.0;
  public static final double VELOCITY_TOLERANCE_RPM = 1.0;
}
