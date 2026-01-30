package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.MatchStateCalculator;

public final class TurretConstants {
  public static final int MOTOR_ID = 5;
  public static final double GEAR_RATIO = 80.0;
  public static final double kP = 2.0;
  public static final double kI = 0.0;
  public static final double kD = 0.05;

  public static final Translation2d HUB_LOCATION = MatchStateCalculator.getHub();
}
