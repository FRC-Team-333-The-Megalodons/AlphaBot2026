package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.interfaces.Initializable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements Initializable {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Supplier<Angle> targetAngleSupplier;
  private final Supplier<Rotation2d> robotRotationSupplier;
  private final SysIdRoutine sysIdRoutine;

  public Turret(
      TurretIO io,
      Supplier<Angle> targetAngleSupplier,
      Supplier<Rotation2d> robotRotationSupplier) {
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
        })
        .until(() -> io.atTarget(targetAngleRelative.getDegrees()));
  }

  public Command toPositive90Deg() {
    return rotateToRelative(Rotation2d.kCW_90deg);
  }

  public Command toNegative90Deg() {
    return rotateToRelative(Rotation2d.kCCW_90deg);
  }

  private Command seedPosition() {
    return Commands.sequence(Commands.waitUntil(() -> io.encodersGood()), reseedPosition())
        .ignoringDisable(true);
  }

  public Command reseedPosition() {
    return Commands.runOnce(() -> io.seedTurretPosition());
  }

  public Command autoAim() {
    return Commands.run(
        () -> {
          Rotation2d targetFieldAngle = new Rotation2d(targetAngleSupplier.get());
          Rotation2d targetRobotAngle = targetFieldAngle.minus(robotRotationSupplier.get());
          // convert from robot frame to turret frame.
          // The turret's mechanical zero is the BACK of the robot, not the front.
          // Adding TURRET_MOUNTING_OFFSET_DEG (180°) shifts the command from
          // "front of robot = 0" into "back of robot = 0"(I F HATED THIS SO MUCH).
          Rotation2d turretTargetAngle =
              targetRobotAngle.plus(
                  Rotation2d.fromDegrees(TurretConstants.TURRET_MOUNTING_OFFSET_DEG));

          double currentDeg = inputs.turretPositionDeg;
          double targetDeg = turretTargetAngle.getDegrees();

          double diff = targetDeg - currentDeg;

          diff = MathUtil.inputModulus(diff, -180, 180);
          double optimalTargetDeg = currentDeg + diff;

          // TODO:When we figure out how to make the turret 360 again -> uncomment this and remove
          // the clamping. We want to be able to rotate the turret more than 180 degrees if needed,
          // we just have to make sure to take the shortest path there.

          // if (optimalTargetDeg > TurretConstants.kMaxAngle) {
          //   optimalTargetDeg -= 360.0;
          // } else if (optimalTargetDeg < TurretConstants.kMinAngle) {
          //   optimalTargetDeg += 360.0;
          // }

          optimalTargetDeg =
              MathUtil.clamp(
                  optimalTargetDeg, TurretConstants.kMinAngle, TurretConstants.kMaxAngle);

          io.moveTo(optimalTargetDeg);
        },
        this);
  }

  public Command rotateToField(Rotation2d targetAngle) {
    return Commands.run(
        () -> {
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
        },
        this);
  }

  // TODO: Play around with the threshold value (currently 4.0 deg) to find the optimal one
  public boolean atTarget() {
    double currentAngle = inputs.turretPositionDeg;

    Rotation2d targetFieldAngle = new Rotation2d(targetAngleSupplier.get());
    Rotation2d targetRobotAngle = targetFieldAngle.minus(robotRotationSupplier.get());
    Rotation2d turretTargetAngle =
        targetRobotAngle.minus(Rotation2d.fromDegrees(TurretConstants.TURRET_MOUNTING_OFFSET_DEG));

    double targetDeg = turretTargetAngle.getDegrees();
    double diff = MathUtil.inputModulus(targetDeg - currentAngle, -180, 180);

    return Math.abs(diff) < 4.0;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
