package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.transfer.Transfer;

public class ShootingCommands {
  static GenericEntry rpmEntry;

  public static Command dashboardRPMControl(Flywheel flywheel) {

    rpmEntry =
        Shuffleboard.getTab("Shooter Tuning")
            .add("Tuning Target RPM", 0.0)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();

    return flywheel.runEnd(
        () -> {
          double targetRPM = rpmEntry.getDouble(0.0);
          flywheel.spinAt(targetRPM, false);
        },
        flywheel::stop);
  }

  /**
   * Automatically aims the turret (Hub or Ferry based on field zone), spins up the flywheel to the
   * dynamic distance, and fires when both are ready.
   */
  public static Command autoAimAndFire(
      Flywheel flywheel, Turret turret, Spindexer spindexer, Transfer transfer, Intake intake) {

    return Commands.parallel(
        turret.autoAim(),
        flywheel.dynamicSpinUp(false),
        Commands.sequence(
            Commands.waitUntil(() -> flywheel.ready() && turret.atTarget()),
            Commands.parallel(spindexer.spin(), transfer.feedShooter(), intake.ingest())));
  }
}
