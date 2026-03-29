package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.interfaces.Characterizable;
import frc.robot.interfaces.Initializable;
import frc.robot.util.LiveTuning;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase implements Characterizable, Initializable {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Supplier<Angle> targetAngleSupplier;
  private final Supplier<Rotation2d> robotRotationSupplier;

  private final DoubleSupplier targetAngularVelocitySupplier;

  private final DoubleSupplier robotOmegaSupplier;

  private final SysIdRoutine sysIdRoutine;

  public Turret(
      TurretIO io,
      Supplier<Angle> targetAngleSupplier,
      Supplier<Rotation2d> robotRotationSupplier,
      DoubleSupplier targetAngularVelocitySupplier,
      DoubleSupplier robotOmegaSupplier) {
    this.io = io;
    this.targetAngleSupplier = targetAngleSupplier;
    this.robotRotationSupplier = robotRotationSupplier;
    this.targetAngularVelocitySupplier = targetAngularVelocitySupplier;
    this.robotOmegaSupplier = robotOmegaSupplier;
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
    // Live values for real-time turret monitoring
    LiveTuning.publish("Turret/PositionDeg", inputs.turretPositionDeg);
    // LiveTuning.publish("Turret/VelocityRPM", inputs.turretVelocityRPM);
    // LiveTuning.publish("Turret/AtTarget", atTarget());
    // LiveTuning.publish("Turret/AbsPositionRot", inputs.calculatedAbsPositionRot);
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
          // --- Step 1: Compute target position (same as before) ---
          Rotation2d targetFieldAngle = new Rotation2d(targetAngleSupplier.get());
          Rotation2d targetRobotAngle = targetFieldAngle.minus(robotRotationSupplier.get());
          double targetDeg = mapToTurretRange(targetRobotAngle.getDegrees());

          double targetAngVelRadPerSec = targetAngularVelocitySupplier.getAsDouble();

          double robotOmegaRadPerSec = robotOmegaSupplier.getAsDouble();

          double turretFeedforwardRadPerSec = targetAngVelRadPerSec - robotOmegaRadPerSec;
          double turretFeedforwardDegPerSec = Units.radiansToDegrees(turretFeedforwardRadPerSec);

          io.moveToWithVelocity(targetDeg, turretFeedforwardDegPerSec);

          Logger.recordOutput("Turret/TargetFieldAngleDeg", targetFieldAngle.getDegrees());
          Logger.recordOutput("Turret/TargetRobotAngleDeg", targetRobotAngle.getDegrees());
          Logger.recordOutput("Turret/MappedTargetDeg", targetDeg);
          Logger.recordOutput("Turret/FeedforwardDegPerSec", turretFeedforwardDegPerSec);
          Logger.recordOutput("Turret/TargetAngVelRadPerSec", targetAngVelRadPerSec);
          Logger.recordOutput("Turret/RobotOmegaRadPerSec", robotOmegaRadPerSec);
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
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setTurretVoltage(voltage.in(Volts)), null, this));

    return Commands.sequence(
        Commands.print("Starting Turret SysId"),
        runSysIdSequence(routine),
        Commands.print("Turret SysId Completed"));
  }
}
