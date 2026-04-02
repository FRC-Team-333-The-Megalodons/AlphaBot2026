package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {
  public static final int MOTOR_ID = 14;
  public static final int MOTOR_2_ID = 9;
  public static final double GEAR_RATIO = 1.0;
  // This is for Bang Bang controller
  // kV_FF = hold_voltage / (target_rpm / 60.0)
  public static final double kV_FF = 0.123;
  public static final double kS_FF = 0.0;
  public static final double kP_TRIM = 0.0;
  // These are for on-board contollers
  public static final double kS = 0.0;
  public static final double kV = 0.3;
  public static final double kA = 0.0;
  public static final double kP = 23.0;

  public static final double MAX_VELOCITY = 100.0;
  public static final double MAX_ACCEL = 200.0;
  public static final double MAX_JERK = 1000.0;

  public static final double VELOCITY_TOLERANCE_RPM = 50.0;
}
