package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
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
          flywheel.setRPMDirect(targetRPM);
        },
        flywheel::stopMotor);
  }

  public static Command autoAimAndFire(
      Flywheel flywheel, Turret turret, Spindexer spindexer, Transfer transfer, Intake intake) {

    return Commands.parallel(
        turret.autoAim(),
        flywheel.dynamicSpinUp(false),
        Commands.sequence(
            Commands.waitUntil(() -> flywheel.ready()),
            Commands.parallel(spindexer.spin(), transfer.feedShooter(), intake.ingest())));
  }

  public static Command shootOnMove(
      Flywheel flywheel,
      Turret turret,
      Spindexer spindexer,
      Transfer transfer,
      Intake intake,
      Pivot pivot,
      Drive drive) {

    return Commands.parallel(
        turret.autoAim(),
        flywheel.shootOnMoveSpinUp(),
        intake.dynamicIngest(
            () -> {
              var fieldVelocity = drive.robotFieldVelocity();
              double absX = Math.abs(fieldVelocity.dx);
              double absY = Math.abs(fieldVelocity.dy);
              return Math.max(absX, absY);
            }),
        Commands.sequence(
            Commands.waitUntil(() -> flywheel.ready() && turret.atTarget()),
            transfer
                .feedShooter()
                .alongWith(
                    spindexer
                        .spin()))); // feedShooterVelocity add this for more consisent shooting.
  }
}
