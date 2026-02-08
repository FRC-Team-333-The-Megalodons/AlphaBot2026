// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;

/** Add your docs here. */
public class AutonomousCommands {
  public static Command shootCommand(
      Drive drive, Flywheel flywheel, Intake intake, Spindexer spindexer, Transfer transfer) {
    double distance = drive.getDistanceToHub();
    double targetRPM = FlywheelConstants.getTargetRPM(distance);
    return Commands.run(
        () ->
            flywheel
                .spinUpCommand(targetRPM)
                .alongWith(
                    Commands.waitUntil(flywheel::isAtSpeed)
                        .andThen(
                            Commands.parallel(
                                intake.runIntakeCommand(),
                                spindexer.activeSpindexerCommand(),
                                transfer.feedShooterCommand()))),
        new Subsystem[] {intake, drive, flywheel, spindexer, transfer});
   }
}
