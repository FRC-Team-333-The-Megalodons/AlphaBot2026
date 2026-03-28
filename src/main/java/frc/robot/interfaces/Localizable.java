package frc.robot.interfaces;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Any Subsystem or IO interface which require commonly-used coordinates, should implement this
 * interface.
 */
interface Localizable {
  static final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public static Distance fieldLength() {
    return Meters.of(tagLayout.getFieldLength());
  }

  public static Distance fieldWidth() {
    return Meters.of(tagLayout.getFieldWidth());
  }

  public static Translation2d fieldCenter() {
    return new Translation2d(fieldLength().div(2.0), fieldWidth().div(2.0));
  }

  public static Pose2d tagPose(int tagId) {
    return tagLayout.getTagPose(tagId).get().toPose2d();
  }

  public static Translation2d tagCoordinates(int tagId) {
    return tagPose(tagId).getTranslation();
  }

  public static Translation2d xUnitVector() {
    return new Translation2d(Meters.of(1.0), Meters.zero());
  }

  public static Translation2d yUnitVector() {
    return new Translation2d(Meters.zero(), Meters.of(1.0));
  }

  public static Translation2d xUnitVector(double magnitude) {
    return new Translation2d(Meters.of(magnitude), Meters.zero());
  }

  public static Translation2d yUnitVector(double magnitude) {
    return new Translation2d(Meters.zero(), Meters.of(magnitude));
  }
}
