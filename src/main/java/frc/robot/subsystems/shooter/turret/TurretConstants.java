package frc.robot.subsystems.shooter.turret;

public class TurretConstants {

  public static final double kTurretZeroOffset = 0.262;

  public static final double kEncoder17ZeroOffset = -0.205;
  public static final double kEncoder18ZeroOffset = -0.793;

  public static final boolean kEncoder17Inverted = true;
  public static final boolean kEncoder18Inverted = true;

  public static final int kTurretMotorId = 7;
  public static final int kEncoder17Id = 44;
  public static final int kEncoder18Id = 43;

  public static final int kTurretGearTeeth = 105;
  public static final int kEncoder1Teeth = 17;
  public static final int kEncoder2Teeth = 18;

  public static final double kMotorToTurretRatio = 19.6875;

  // Limits(Might Change)

  // TEMPORARY (HOPEFULLY): The turret only has 180 degrees of freedom right now (big sad)
  //  due to wiring issues. So we'll have to limit the motion appropriately here.
  // TODO: Figure out what these actual numbers are from physically moving the robot and monitoring
  // AdvantageKit.
  public static final double kMinAngle = 0.0; // -179.99999;
  public static final double kMaxAngle = 180.0;

  public static final double kP = 20;
  public static final double kI = 0.0;
  public static final double kD = 0.052;
  public static final double kS = 0.65;
  public static final double kV = 0.12;
  public static final double kA = 0.01;

  public static final double kCruiseVelocity = 4.0;
  public static final double kAcceleration = 8.0;
  public static final double kJerk = 80.0;

  public static final double positionTolerance = 2.5;
  public static final double velocityTolerance = 5.0;
}
