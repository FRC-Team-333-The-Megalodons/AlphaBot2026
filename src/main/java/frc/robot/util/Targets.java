package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.interfaces.Localizable;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Helper class with a list of targets for the turret and lywheel to aim at. */
public class Targets implements Localizable {

  private Map<String, List<Translation2d>> targets;

  public Targets() {
    targets = new HashMap<>();
    configureTargetMap();
  }

  private void addPoints(String targetName, Translation2d... points) {
    targets.put(targetName, Arrays.asList(points));
  }

  public Translation2d select(String targetName) {
    return fieldCenter().nearest(targets.get(targetName));
  }

  public Translation2d select(String targetName, Pose2d robotPose) {
    return robotPose.getTranslation().nearest(targets.get(targetName));
  }

  private void configureTargetMap() {
    addPoints("redHub", tagCoordinates(10).plus(tagCoordinates(4)).div(2));

    addPoints("blueHub", tagCoordinates(25).plus(tagCoordinates(20)).div(2));

    Translation2d ferryXOffset = xUnitVector();
    Translation2d ferryYOffset = yUnitVector().div(4);

    addPoints(
        "redZone",
        tagCoordinates(6).plus(ferryXOffset).plus(ferryYOffset),
        tagCoordinates(1).plus(ferryXOffset).minus(ferryYOffset));

    addPoints(
        "blueZone",
        tagCoordinates(17).minus(ferryXOffset).plus(ferryYOffset),
        tagCoordinates(22).minus(ferryXOffset).minus(ferryYOffset));

    Translation2d neutralYOffset = yUnitVector().times(3);

    addPoints(
        "neutralZone", fieldCenter().plus(neutralYOffset), fieldCenter().minus(neutralYOffset));
  }
}
