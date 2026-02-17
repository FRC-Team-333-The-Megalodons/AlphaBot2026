package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldLayout;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Supplier<Pose2d> robotPoseSupplier;

  public Turret(TurretIO io, Supplier<Pose2d> robotPoseSupplier) {
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command setVoltage(double volts) {
    return Commands.runEnd(() -> io.setTurretVoltage(volts), () -> io.stop(), this);
  }

  public boolean isAtPositive90() {
    return inputs.turretPositionRad > 1.4 && inputs.turretPositionRad < 1.6 ? true : false;
  }

  public boolean isAtNegative90() {
    return inputs.turretPositionRad < -1.4 && inputs.turretPositionRad > -1.6 ? true : false;
  }

  public Command setTo90Deg() {
    return setVoltage(3).until(() -> isAtPositive90());
  }
  public Command aimAtFieldZero() {
    return Commands.run(
        () -> {
          Pose2d robotPose = robotPoseSupplier.get();

          Rotation2d targetFieldAngle = Rotation2d.fromDegrees(0);

          Rotation2d targetRobotAngle = targetFieldAngle.minus(robotPose.getRotation());

          double currentDeg = Math.toDegrees(inputs.turretPositionRad);
          double targetDeg = targetRobotAngle.getDegrees();

          double diff = targetDeg - currentDeg;

          diff = MathUtil.inputModulus(diff, -180, 180);
          double optimalTargetDeg = currentDeg + diff;

          if (optimalTargetDeg > TurretConstants.kMaxAngle) {
            optimalTargetDeg -= 360.0;
          } else if (optimalTargetDeg < TurretConstants.kMinAngle) {
            optimalTargetDeg += 360.0;
          }

          optimalTargetDeg =
              MathUtil.clamp(
                  optimalTargetDeg, TurretConstants.kMinAngle, TurretConstants.kMaxAngle);

          io.setTurretPosition(Rotation2d.fromDegrees(optimalTargetDeg));
        },
        this);
  }

  public Command aimAtHub() {
    return Commands.run(
        () -> {
          Pose2d robotPose = robotPoseSupplier.get();
          Translation2d hubLoc = FieldLayout.getStaticHub();

          double dx = hubLoc.getX() - robotPose.getX();
          double dy = hubLoc.getY() - robotPose.getY();
          Rotation2d targetFieldAngle = new Rotation2d(Math.atan2(dy, dx));

          Rotation2d targetRobotAngle = targetFieldAngle.minus(robotPose.getRotation());

          double currentDeg = Math.toDegrees(inputs.turretPositionRad);
          double targetDeg = targetRobotAngle.getDegrees();

          double diff = targetDeg - currentDeg;

          diff = MathUtil.inputModulus(diff, -180, 180);
          double optimalTargetDeg = currentDeg + diff;

          if (optimalTargetDeg > TurretConstants.kMaxAngle) {
            optimalTargetDeg -= 360.0;
          } else if (optimalTargetDeg < TurretConstants.kMinAngle) {
            optimalTargetDeg += 360.0;
          }

          optimalTargetDeg =
              MathUtil.clamp(
                  optimalTargetDeg, TurretConstants.kMinAngle, TurretConstants.kMaxAngle);

          io.setTurretPosition(Rotation2d.fromDegrees(optimalTargetDeg));
        },
        this);
  }
}
