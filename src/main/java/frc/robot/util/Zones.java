package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.interfaces.Localizable;
import java.util.function.Predicate;

public class Zones implements Localizable {

  private Distance redZoneLine, blueZoneLine;
  private Predicate<Distance> inRedZone, inNeutralZone, inBlueZone;

  public Zones() {
    redZoneLine = tagCoordinates(6).plus(tagCoordinates(7)).div(2.0).getMeasureX();
    blueZoneLine = tagCoordinates(22).plus(tagCoordinates(23)).div(2.0).getMeasureX();

    inRedZone = (xDistance) -> xDistance.gte(redZoneLine) && xDistance.lte(fieldLength());
    inNeutralZone = (xDistance) -> xDistance.gte(blueZoneLine) && xDistance.lte(redZoneLine);
    inBlueZone = (xDistance) -> xDistance.gte(Meters.zero()) && xDistance.lte(blueZoneLine);
  }

  public boolean alliance(Pose2d robotPose) {
    Distance x = robotPose.getMeasureX();

    return DriverStation.getAlliance().get() == Alliance.Blue
        ? inBlueZone.test(x)
        : inRedZone.test(x);
  }

  public boolean enemy(Pose2d robotPose) {
    Distance x = robotPose.getMeasureX();

    return DriverStation.getAlliance().get() == Alliance.Blue
        ? inRedZone.test(x)
        : inBlueZone.test(x);
  }

  public boolean neutral(Pose2d robotPose) {
    return inNeutralZone.test(robotPose.getMeasureX());
  }
}
