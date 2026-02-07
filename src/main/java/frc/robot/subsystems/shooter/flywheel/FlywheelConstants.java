package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final int MOTOR_ID = 6;
  public static final int MOTOR_2_ID = 9;
  public static final double GEAR_RATIO = 1.0;
  public static final double kV = 0.12;
  public static final double kP = 0.1;
  public static final double VELOCITY_TOLERANCE_RPM = 100.0;

  public static double getTargetRPM(double distanceMeters) {
    if (distanceMeters < 2.0) {
      return -2000.0;
    } else if (distanceMeters < 2.5) {
      return -2250.0;
    } else if (distanceMeters < 3.0) {
      return -2500.0;
    } else if (distanceMeters < 3.5) {
      return -2650.0;
    } else if (distanceMeters < 4.0) {
      return -2700.0;
    } else {
      return -3500.0;
    }
  }
}
