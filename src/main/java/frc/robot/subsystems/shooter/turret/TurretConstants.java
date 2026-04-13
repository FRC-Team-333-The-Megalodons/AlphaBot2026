package frc.robot.subsystems.shooter.turret;

public class TurretConstants {

  public static final double kTurretZeroOffset = 0.0;
  // Must be negative values
  public static final double kEncoder17ZeroOffset = -0.353;
  public static final double kEncoder18ZeroOffset = -0.990;

  public static final boolean kEncoder17Inverted = true;
  public static final boolean kEncoder18Inverted = true;

  public static final int kTurretMotorId = 7;
  public static final int kEncoder17Id = 44;
  public static final int kEncoder18Id = 43;

  public static final int kTurretGearTeeth = 105;
  public static final int kEncoder1Teeth = 17;
  public static final int kEncoder2Teeth = 18;

  public static final double kMotorToTurretRatio = 19.6875;

  public static final double kMinAngle = -284.0; // Changed from -220.0
  public static final double kMaxAngle = 103.0;

  public static final double kP = 45.8;
  public static final double kI = 0.75;
  public static final double kD = 1.9;
  public static final double kS = 0.65;
  public static final double kV = 0.12;
  public static final double kA = 0.01;

  public static final double kCruiseVelocity = 0.75;
  public static final double kAcceleration = 2.25;
  public static final double kJerk = 9;

  public static final double positionTolerance = 0.7;
  public static final double velocityTolerance = 5.0;
}
