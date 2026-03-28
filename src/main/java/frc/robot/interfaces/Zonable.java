package frc.robot.interfaces;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Predicate;

public interface Zonable extends Localizable {

  static final Distance redZoneLine =
      Localizable.tagCoordinates(6).plus(Localizable.tagCoordinates(7)).div(2.0).getMeasureX();

  static final Distance blueZoneLine =
      Localizable.tagCoordinates(22).plus(Localizable.tagCoordinates(23)).div(2.0).getMeasureX();

  static final Predicate<Distance> inRedZone =
      (xDistance) -> xDistance.gt(redZoneLine) && xDistance.lte(Localizable.fieldLength());
  static final Predicate<Distance> inNeutralZone =
      (xDistance) -> xDistance.gte(blueZoneLine) && xDistance.lte(redZoneLine);
  static final Predicate<Distance> inBlueZone =
      (xDistance) -> xDistance.gte(Meters.zero()) && xDistance.lt(blueZoneLine);

  public default boolean inAllianceZone(Pose2d robotPose) {
    Distance x = robotPose.getMeasureX();

    return Localizable.alliance() == Alliance.Red ? inRedZone.test(x) : inBlueZone.test(x);
  }

  public default boolean inEnemyZone(Pose2d robotPose) {
    Distance x = robotPose.getMeasureX();

    return Localizable.alliance() == Alliance.Red ? inBlueZone.test(x) : inRedZone.test(x);
  }

  public default boolean inNeutralZone(Pose2d robotPose) {
    return inNeutralZone.test(robotPose.getMeasureX());
  }
}
