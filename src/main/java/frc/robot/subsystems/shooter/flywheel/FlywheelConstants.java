package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final int MOTOR_ID = 14;
  public static final int MOTOR_2_ID = 9;
  public static final double GEAR_RATIO = 1.0;

  public static final double kS = 0.21636;
  public static final double kV = 0.11765;
  // public static final double kP = 0.019414;
  public static final double kA = 0.015143;
  public static final double kP = 0.067809;

  public static final double MAX_ACCEL = 200.0;
  public static final double MAX_JERK = 1000.0;

  public static final double VELOCITY_TOLERANCE_RPM = 50.0;

  public static double getTargetRPM(double distanceMeters) {
    if (distanceMeters < 2.0) return -1800.0;
    if (distanceMeters < 2.65) return -1950.0;
    if (distanceMeters < 3.0) return -2100.0;
    if (distanceMeters < 3.78) return -2300.0;
    if (distanceMeters < 4.5) return -2600.0;
    if (distanceMeters < 5.5) return -2700.0;
    return -1500;
  }
}
