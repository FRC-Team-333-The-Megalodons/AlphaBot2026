package frc.robot.commands;

import static frc.robot.AutopilotConstants.kClimbingAutopilot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldLayout;
import frc.robot.util.MatchStateCalculator;
import java.util.Set;

public class PathfindCommands {

  public static Command pathfindTo(
      Translation2d translation, Rotation2d rotation, Drive drive, double endVelocity) {
    Pose2d targetPose = new Pose2d(translation, rotation);
    PathConstraints constraints =
        new PathConstraints(2.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(targetPose, constraints, endVelocity);
  }

  public static Command precisionPathfindTo(Pose2d targetPose2d, Drive drive) {
    Pose2d handoffPose =
        targetPose2d.transformBy(
            new edu.wpi.first.math.geometry.Transform2d(
                new Translation2d(-0.5, 0), new Rotation2d(0)));
    Command pathfindTo =
        pathfindTo(handoffPose.getTranslation(), handoffPose.getRotation(), drive, 1.5);
    Command autoPilot = new DriveToPose(drive, targetPose2d);

    return pathfindTo.andThen(autoPilot);
  }

  public static Command pathfindToDepot(Drive drive) {
    return pathfindTo(
        FieldLayout.Depot.DEPOT_SCORING_POSITION.getTranslation(),
        FieldLayout.Depot.DEPOT_SCORING_POSITION.getRotation(),
        drive,
        0.0);
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
        drive,
        0.0);
  }

  public static Command pathfindtoScoringPosition(Drive drive) {
    return pathfindTo(
        FieldLayout.ScoringPosition.SCORING_POSITION_A.getTranslation(),
        FieldLayout.ScoringPosition.SCORING_POSITION_A.getRotation(),
        drive,
        0.0);
  }

  
  public static Command climbSequence(Drive drive) {
    PathConstraints stagingConstraints =
        new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return Commands.sequence(
           
            AutoBuilder.pathfindToPose(
                FieldLayout.Tower.CLIMBING_STAGING_POSE, stagingConstraints, 0.0),
            Commands.waitSeconds(0.3),
            Commands.defer(
                () -> DriveCommands.driveToPose(drive, allianceClimbingPose(), kClimbingAutopilot),
                Set.of(drive)))
        .withName("PathfindCommands.climbSequence");
  }
  private static Pose2d allianceClimbingPose() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? FlippingUtil.flipFieldPose(FieldLayout.Tower.CLIMBING_POSE)
        : FieldLayout.Tower.CLIMBING_POSE;
  }
}
