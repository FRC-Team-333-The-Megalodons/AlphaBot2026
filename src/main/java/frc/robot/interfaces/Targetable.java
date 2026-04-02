package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public interface Targetable extends Localizable {

  @FunctionalInterface
  public interface TargetsConsumer {
    void accept(String targetName, Translation2d... points);
  }

  static final Map<String, List<Translation2d>> targets = configureTargetMap();

  /**
   * Select a target. Use this for sole targets like the hub.
   *
   * @param targetName The name of the target.
   * @return The target's position via a <code>Translation2d</code>.
   */
  public default Translation2d select(String targetName) {
    return Localizable.fieldCenter().nearest(targets.get(targetName));
  }

  /**
   * Select a target. Use this for selecting optimal targets, such as which side of a zone to ferry
   * to.
   *
   * @param targetName The name of the target.
   * @param robotPose A <code>Pose2d</code> representing the Robot's current position.
   * @return The target's position via a <code>Translation2d</code>.
   */
  public default Translation2d select(String targetName, Pose2d robotPose) {
    return robotPose.getTranslation().nearest(targets.get(targetName));
  }

  /**
   * Used to set up the Target Map.
   *
   * @return The configured Target Map.
   */
  private static HashMap<String, List<Translation2d>> configureTargetMap() {
    HashMap<String, List<Translation2d>> targetMap = new HashMap<>();

    TargetsConsumer addTarget =
        (targetName, points) -> targetMap.put(targetName, Arrays.asList(points));

    addTarget.accept(
        "redHub", Localizable.tagCoordinates(10).plus(Localizable.tagCoordinates(4)).div(2));

    addTarget.accept(
        "blueHub", Localizable.tagCoordinates(25).plus(Localizable.tagCoordinates(19)).div(2));

    Translation2d ferryXOffset = Localizable.xUnitVector(2.5);
    // Translation2d ferryYOffset = yUnitVector().div(4);
    Translation2d ferryYOffset = Localizable.yUnitVector(1.3);

    addTarget.accept(
        "redZone",
        Localizable.tagCoordinates(6).plus(ferryXOffset).plus(ferryYOffset),
        Localizable.tagCoordinates(1).plus(ferryXOffset).minus(ferryYOffset));

    addTarget.accept(
        "blueZone",
        Localizable.tagCoordinates(17).minus(ferryXOffset).plus(ferryYOffset),
        Localizable.tagCoordinates(22).minus(ferryXOffset).minus(ferryYOffset));

    Translation2d neutralYOffset = Localizable.yUnitVector(3);

    addTarget.accept(
        "neutralZone",
        Localizable.fieldCenter().plus(neutralYOffset),
        Localizable.fieldCenter().minus(neutralYOffset));

    return targetMap;
  }
}
