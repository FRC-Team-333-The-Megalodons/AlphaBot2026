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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.interfaces.Characterizable;
import frc.robot.interfaces.Initializable;
import frc.robot.util.LiveTuning;
import frc.robot.util.RobotMetrics;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements Characterizable, Initializable {
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
                (state) -> RobotMetrics.recordOutput("Turret/SysIdState", state.toString())),
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
    // Live values for real-time turret monitoring
    LiveTuning.publish("Turret/PositionDeg", inputs.turretPositionDeg);
    LiveTuning.publish("Turret/VelocityRPM", inputs.turretVelocityRPM);
    LiveTuning.publish("Turret/AtTarget", atTarget());
    LiveTuning.publish("Turret/AbsPositionRot", inputs.calculatedAbsPositionRot);
    LiveTuning.publish("Turret/Encoder17", inputs.encoder17Rotations);
    LiveTuning.publish("Turret/Encoder18", inputs.encoder18Rotations);
  }

  public Command setVoltage(double volts) {
    return Commands.runEnd(() -> io.setTurretVoltage(volts), () -> io.stop(), this);
  }

  public Command rotateToRelative(Rotation2d targetAngleRelative) {
    return run(() -> io.moveTo(targetAngleRelative.getDegrees()))
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

  private double mapToTurretRange(double targetDeg) {

    if (targetDeg < TurretConstants.kMinAngle) {
      targetDeg += 360.0;
    }
    return MathUtil.clamp(targetDeg, TurretConstants.kMinAngle, TurretConstants.kMaxAngle);
  }

  public Command autoAim() {
    return Commands.run(
        () -> {
          Rotation2d targetFieldAngle = new Rotation2d(targetAngleSupplier.get());

          Rotation2d targetRobotAngle = targetFieldAngle.minus(robotRotationSupplier.get());

          double targetDeg = mapToTurretRange(targetRobotAngle.getDegrees());

          RobotMetrics.recordOutput("Turret/TargetFieldAngleDeg", targetFieldAngle.getDegrees());
          RobotMetrics.recordOutput("Turret/TargetRobotAngleDeg", targetRobotAngle.getDegrees());
          RobotMetrics.recordOutput("Turret/MappedTargetDeg", targetDeg);

          io.moveTo(targetDeg);
        },
        this);
  }

  public Command rotateToField(Rotation2d targetAngle) {
    return Commands.run(
        () -> {
          double targetDeg = mapToTurretRange(targetAngle.getDegrees());
          io.moveTo(targetDeg);
        },
        this);
  }

  public boolean atTarget() {
    double currentAngle = inputs.turretPositionDeg;

    Rotation2d targetFieldAngle = new Rotation2d(targetAngleSupplier.get());
    Rotation2d targetRobotAngle = targetFieldAngle.minus(robotRotationSupplier.get());
    double targetDeg = mapToTurretRange(targetRobotAngle.getDegrees());

    double diff = MathUtil.inputModulus(targetDeg - currentAngle, -180, 180);

    return Math.abs(diff) < 4.0;
  }

  @Override
  public Command characterize() {
    SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        (state) -> RobotMetrics.recordOutput("Turret/SysIdState", state.toString())),
      new SysIdRoutine.Mechanism(
        (voltage) -> io.setTurretVoltage(voltage.in(Volts)),
        null,
        this
      )
    );

    return Commands.sequence(
      Commands.print("Starting Turret SysId"),
      runSysIdSequence(routine),
      Commands.print("Turret SysId Completed")
    );
  }
}
