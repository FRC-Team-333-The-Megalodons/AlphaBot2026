package frc.robot.subsystems.shooter.turret;

public class TurretConstants {

  public static final double kTurretZeroOffset = 0.254;

  public static final int kTurretMotorId = 7;
  public static final int kEncoder17Id = 44;
  public static final int kEncoder18Id = 43;

  public static final double kTurretGearTeeth = 105.0;
  public static final double kEncoder1Teeth = 17.0;
  public static final double kEncoder2Teeth = 18.0;

  public static final double kMotorToTurretRatio = 19.6875;

  // Limits(Might Change)
  public static final double kMinAngle = -180.0;
  public static final double kMaxAngle = 180.0;

  public static final double kP = 10;
  public static final double kI = 0.0;
  public static final double kD = 0.1;
  public static final double kS = 0.4;
  public static final double kV = 0.12;
  public static final double kA = 0.01;

  public static final double kCruiseVelocity = 4.0;
  public static final double kAcceleration = 8.0;
  public static final double kJerk = 80.0;
}
