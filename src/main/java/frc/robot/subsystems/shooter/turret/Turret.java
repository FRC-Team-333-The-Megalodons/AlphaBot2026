package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command trackHubCommand(Supplier<Pose2d> robotPose) {
    return run(
        () -> {
          Translation2d robotTrans = robotPose.get().getTranslation();
          Rotation2d angleToHub = TurretConstants.HUB_LOCATION.minus(robotTrans).getAngle();
          Rotation2d robotRelative = angleToHub.minus(robotPose.get().getRotation());
          io.setPosition(robotRelative.getRadians());
        });
  }

  public void setAngle(Rotation2d angle) {
    io.setPosition(angle.getRadians());
  }

  public Command trackHub(Supplier<Pose2d> robotPose) {
    return run(
        () -> {
          Translation2d robotTrans = robotPose.get().getTranslation();
          Rotation2d angleToHub = TurretConstants.HUB_LOCATION.minus(robotTrans).getAngle();
          Rotation2d robotRelative = angleToHub.minus(robotPose.get().getRotation());
          io.setPosition(robotRelative.getRadians());
        });
  }
}
