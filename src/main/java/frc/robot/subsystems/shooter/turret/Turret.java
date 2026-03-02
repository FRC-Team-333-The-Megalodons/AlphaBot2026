package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.interfaces.Initializable;
import frc.robot.util.MatchStateCalculator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;

public class Turret extends SubsystemBase implements Initializable {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Supplier<Angle> targetAngleSupplier;
  private final Supplier<Rotation2d> robotRotationSupplier;
  private final SysIdRoutine sysIdRoutine;

  public Turret(TurretIO io, Supplier<Angle> targetAngleSupplier, Supplier<Rotation2d> robotRotationSupplier) {
    this.io = io;
    this.targetAngleSupplier = targetAngleSupplier;
    this.robotRotationSupplier = robotRotationSupplier;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setTurretVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void seed() {
    CommandScheduler.getInstance().schedule(seedPosition());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command setVoltage(double volts) {
    return Commands.runEnd(() -> io.setTurretVoltage(volts), () -> io.stop(), this);
  }

  /**
   * Debugging Command to test relative angle rotation of turret.
   * 
   * @param targetAngleRelative The relative target angle.
   * @return A command.
   */
  public Command rotateToRelative(Rotation2d targetAngleRelative) {
    return run(() -> {
      io.moveTo(targetAngleRelative.getDegrees());
    }).until(() -> io.atTarget(targetAngleRelative.getDegrees()));
  }

  public Command toPositive90Deg() {
    return rotateToRelative(Rotation2d.kCW_90deg);
  }

  public Command toNegative90Deg() {
    return rotateToRelative(Rotation2d.kCCW_90deg);
  }

  private Command seedPosition() {
    return Commands.sequence(
      Commands.waitUntil(() -> io.encodersGood()),
      reseedPosition()
    ).ignoringDisable(true);
  }

  public Command reseedPosition() {
    return Commands.runOnce(() -> io.seedTurretPosition());
  }

  /*
  public Command aimAtFieldZero() {
    return Commands.run(
        () -> {
          Pose2d robotPose = robotPoseSupplier.get();

          Rotation2d targetFieldAngle = Rotation2d.fromDegrees(0);

          Rotation2d targetRobotAngle = targetFieldAngle.minus(robotPose.getRotation());

          double currentDeg = inputs.turretPositionDeg;
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

          io.moveTo(optimalTargetDeg);
        },
        this);
  }
  */

  /**
   * Debugging Command to test global field angle rotation of turret.
   * 
   * @param targetAngleRelative The global target angle.
   * @return A command.
   */
  public Command rotateToField(Rotation2d targetAngle) {
    return Commands.run(() -> {
      double currentDeg = inputs.turretPositionDeg;
      double targetDeg = targetAngle.getDegrees();

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

      io.moveTo(optimalTargetDeg);
    }, this);
  }

  public Command aimAtPoint() {
    return Commands.run(() -> {

      Rotation2d targetFieldAngle = new Rotation2d(targetAngleSupplier.get());
      Rotation2d targetRobotAngle = targetFieldAngle.minus(robotRotationSupplier.get());

      double currentDeg = inputs.turretPositionDeg;
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

      io.moveTo(optimalTargetDeg);
    }, this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
