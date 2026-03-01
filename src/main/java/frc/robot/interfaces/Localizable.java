package frc.robot.interfaces;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public interface Localizable {
  static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public default Distance fieldLength()                   { return Meters.of(tagLayout.getFieldLength()); }
  public default Distance fieldWidth()                    { return Meters.of(tagLayout.getFieldWidth()); }
  public default Translation2d fieldCenter()              { return new Translation2d(fieldLength().div(2.0), fieldWidth().div(2.0)); }
  public default Pose2d tagPose(int tagId)                { return tagLayout.getTagPose(tagId).get().toPose2d(); }
  public default Translation2d tagCoordinates(int tagId)  { return tagPose(tagId).getTranslation(); }
  public default Translation2d xUnitVector()              { return new Translation2d(Meters.of(1.0), Meters.zero()); }
  public default Translation2d yUnitVector()              { return new Translation2d(Meters.zero(), Meters.of(1.0)); }
}
