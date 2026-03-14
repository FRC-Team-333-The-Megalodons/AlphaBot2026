package frc.robot.subsystems.climber;

public final class ClimberConstants {

  public static final int MOTOR_ID = 15;

  public static final int LIMIT_SWITCH_CHANNEL = 0;

  public static final double GEAR_RATIO = 45.0;

  // PID — tune these on the real robot
  public static final double kP = 80.0;
  public static final double kI = 0.0;
  public static final double kD = 2.0;
  public static final double kS = 0.3;
  public static final double kV = 0.12;
  public static final double kA = 0.01;
  public static final double kG = 0.0;

  public static final double kCruiseVelocity = 2.0;
  public static final double kAcceleration = 4.0;
  public static final double kJerk = 40.0;

  // Soft limits
  public static final double kMinPositionRot = 0.0;
  public static final double kMaxPositionRot = 20.0; // TODO: measure real max

  // Tolerance for atTarget check
  public static final double POSITION_TOLERANCE_ROT = 0.05;
  public static final double VELOCITY_TOLERANCE_RPS = 0.1;

  // Named positions in mechanism rotations
  public static final double kStowedPosition = 0.0;
  public static final double kClimbPosition = 18.0; // TODO: measure real climb height
}
