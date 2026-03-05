// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Targeting.Targeting;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.FieldLayout;

/** Add your docs here. */
public class AutonomousCommands {

  public static Command shootCommand(
      Drive drive, Flywheel flywheel, Intake intake, Spindexer spindexer, Transfer transfer) {
    return Commands.sequence(
        flywheel.dynamicSpinUp(true),
        Commands.parallel(intake.ingest(), spindexer.spin(), transfer.feedShooter()));
  }

  public static Command pathfindToTower(Drive drive) {
    Pose2d targetPose = new Pose2d(new Translation2d(14.88, 4.2), Rotation2d.kCW_90deg);
    PathConstraints constraints =
        new PathConstraints(1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  public static Command movingShootCommand(
      Drive drive,
      Flywheel flywheel,
      Targeting targeting,
      Turret turret,
      Intake intake,
      Spindexer spindexer,
      Transfer transfer) {

    return Commands.deadline(
        Commands.sequence(
            Commands.deadline(flywheel.dynamicSpinUp(true), turret.autoAim()),
            Commands.parallel(spindexer.spin(), transfer.feedShooter())),
        targeting.simpleTargeting());
  }

  public static Command outpostToHubSequence(
      Drive drive,
      Flywheel flywheel,
      Targeting targeting,
      Turret turret,
      Intake intake,
      Spindexer spindexer,
      Transfer transfer) {

    return Commands.sequence(
        movingShootCommand(drive, flywheel, targeting, turret, intake, spindexer, transfer)
            .withTimeout(2.0),
        PathfindCommands.precisionPathfindTo(FieldLayout.Outpost.OUTPOST_POSE, drive),
        movingShootCommand(drive, flywheel, targeting, turret, intake, spindexer, transfer)
            .withTimeout(3.5),
        PathfindCommands.precisionPathfindTo(FieldLayout.Hub.NEAR_FACE, drive));
  }
}
