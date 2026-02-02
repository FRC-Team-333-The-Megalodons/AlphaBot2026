package frc.robot.subsystems.turret;

public class TurretConstants {

  public static final double kTurretZeroOffset = 0.0;

  public static final int kTurretMotorId = 15;
  public static final int kEncoder17Id = 16;
  public static final int kEncoder18Id = 17;

  public static final double kTurretGearTeeth = 105.0;
  public static final double kEncoder1Teeth = 17.0;
  public static final double kEncoder2Teeth = 18.0;

  public static final double kMotorToTurretRatio =
      kTurretGearTeeth / 10.0; 

  // Limits(Might Change)
  public static final double kMinAngle = -300.0;
  public static final double kMaxAngle = 300.0;

  public static final double kP = 4.0;
  public static final double kI = 0.0;
  public static final double kD = 0.1;
  public static final double kS = 0.25; 
  public static final double kV = 0.12; 
  public static final double kA = 0.01; 

  public static final double kCruiseVelocity = 4.0; 
  public static final double kAcceleration = 8.0; 
  public static final double kJerk = 80.0; 
}
