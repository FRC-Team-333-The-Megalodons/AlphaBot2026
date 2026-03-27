package frc.robot.subsystems.tracker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.Zonable;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class RobotStateTracker extends SubsystemBase implements Zonable {

  // --- Enums ---

  public enum FieldZone {
    ALLIANCE_ZONE,
    NEUTRAL_ZONE,
    ENEMY_ZONE
  }

  public enum ShooterState {
    IDLE,
    SPINNING_UP,
    READY
  }

  public enum GamePieceState {
    NONE,
    INTAKING
  }

  public enum MatchMode {
    DISABLED,
    AUTO,
    TELEOP,
    TEST
  }

  @AutoLog
  public static class RobotStateTrackerInputs {
    public String fieldZone = FieldZone.ALLIANCE_ZONE.name();
    public String shooterState = ShooterState.IDLE.name();
    public String gamePieceState = GamePieceState.NONE.name();
    public String matchMode = MatchMode.DISABLED.name();
  }

  private final RobotStateTrackerInputsAutoLogged inputs = new RobotStateTrackerInputsAutoLogged();

  private final Supplier<Pose2d> poseSupplier;
  private final BooleanSupplier flywheelReady;
  private final BooleanSupplier flywheelSpinningUp;
  private final DoubleSupplier intakeVoltage;

  public RobotStateTracker(
      Supplier<Pose2d> poseSupplier,
      BooleanSupplier flywheelReady,
      BooleanSupplier flywheelSpinningUp,
      DoubleSupplier intakeVoltage) {
    this.poseSupplier = poseSupplier;
    this.flywheelReady = flywheelReady;
    this.flywheelSpinningUp = flywheelSpinningUp;
    this.intakeVoltage = intakeVoltage;
  }

  @Override
  public void periodic() {
    Pose2d pose = poseSupplier.get();

    // Field zone
    if (inAllianceZone(pose)) {
      inputs.fieldZone = FieldZone.ALLIANCE_ZONE.name();
    } else if (inEnemyZone(pose)) {
      inputs.fieldZone = FieldZone.ENEMY_ZONE.name();
    } else {
      inputs.fieldZone = FieldZone.NEUTRAL_ZONE.name();
    }

    // Shooter state
    if (flywheelReady.getAsBoolean()) {
      inputs.shooterState = ShooterState.READY.name();
    } else if (flywheelSpinningUp.getAsBoolean()) {
      inputs.shooterState = ShooterState.SPINNING_UP.name();
    } else {
      inputs.shooterState = ShooterState.IDLE.name();
    }

    // Game piece state
    if (Math.abs(intakeVoltage.getAsDouble()) > 0.1) {
      inputs.gamePieceState = GamePieceState.INTAKING.name();
    } else {
      inputs.gamePieceState = GamePieceState.NONE.name();
    }

    // Match mode
    if (DriverStation.isDisabled()) {
      inputs.matchMode = MatchMode.DISABLED.name();
    } else if (DriverStation.isAutonomous()) {
      inputs.matchMode = MatchMode.AUTO.name();
    } else if (DriverStation.isTeleop()) {
      inputs.matchMode = MatchMode.TELEOP.name();
    } else {
      inputs.matchMode = MatchMode.TEST.name();
    }

    Logger.processInputs("RobotStateTracker", inputs);
  }

  public FieldZone getFieldZone() {
    return FieldZone.valueOf(inputs.fieldZone);
  }

  public ShooterState getShooterState() {
    return ShooterState.valueOf(inputs.shooterState);
  }

  public GamePieceState getGamePieceState() {
    return GamePieceState.valueOf(inputs.gamePieceState);
  }

  public MatchMode getMatchMode() {
    return MatchMode.valueOf(inputs.matchMode);
  }

  public boolean isInAllianceZone() {
    return getFieldZone() == FieldZone.ALLIANCE_ZONE;
  }

  public boolean isInNeutralZone() {
    return getFieldZone() == FieldZone.NEUTRAL_ZONE;
  }

  public boolean isInEnemyZone() {
    return getFieldZone() == FieldZone.ENEMY_ZONE;
  }

  public boolean isShooterReady() {
    return getShooterState() == ShooterState.READY;
  }

  public boolean isShooterSpinningUp() {
    return getShooterState() == ShooterState.SPINNING_UP;
  }

  public boolean isIntaking() {
    return getGamePieceState() == GamePieceState.INTAKING;
  }

  public boolean isAuto() {
    return getMatchMode() == MatchMode.AUTO;
  }

  public boolean isTeleop() {
    return getMatchMode() == MatchMode.TELEOP;
  }

  public boolean isDisabled() {
    return getMatchMode() == MatchMode.DISABLED;
  }
}
