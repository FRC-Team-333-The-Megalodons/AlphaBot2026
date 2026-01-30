package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldLayout;
import frc.robot.util.MatchStateCalculator;

public class PathfindCommands {
  public static Command pathfindTo(Translation2d translation, Rotation2d rotation, Drive drive) {
    Pose2d targetPose = new Pose2d(translation, rotation);
    PathConstraints constraints =
        new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  public static Command pathfindToDepot(Drive drive) {
    return pathfindTo(FieldLayout.Depot.DEPOT_CENTER, Rotation2d.k180deg, drive);
  }

  public static Command pathfindToHub(Drive drive) {
    double x =
        MatchStateCalculator.isBlueAlliance()
            ? FieldLayout.Hub.NEAR_FACE.getX() - 0.7
            : FieldLayout.Hub.NEAR_FACE.getX() + 0.7;
    Translation2d targetTranslation = new Translation2d(x, FieldLayout.Hub.NEAR_FACE.getY());
    return pathfindTo(
        targetTranslation,
        MatchStateCalculator.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg,
        drive);
  }
}
