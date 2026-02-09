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
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;

/** Add your docs here. */
public class AutonomousCommands {
  public static Command shootCommand(
      Drive drive, Flywheel flywheel, Intake intake, Spindexer spindexer, Transfer transfer) {
    return Commands.deferredProxy(
        () -> {
          double distance = drive.getDistanceToHub();
          double targetRPM = flywheel.getRPMForDistance(distance);

          return flywheel
              .spinUpCommand(targetRPM)
              .alongWith(
                  Commands.waitUntil(flywheel::isAtSpeed)
                      .andThen(
                          Commands.parallel(
                              intake.runIntakeCommand(),
                              spindexer.activeSpindexerCommand(),
                              transfer.feedShooterCommand())));
        });
  }

  public static Command pathfindToTower(Drive drive) {
    Pose2d targetPose = new Pose2d(new Translation2d(14.88, 4.2), Rotation2d.kCW_90deg);
    PathConstraints constraints =
        new PathConstraints(1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }
}
