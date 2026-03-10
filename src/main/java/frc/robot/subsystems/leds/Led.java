package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.util.RobotStates;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {

  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

  private LedState currentState = LedState.IDLE;

  public Led(LedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Led", inputs);
  }

 

  private void setState(LedState state) {
    currentState = state;
    inputs.currentState = state.name();
    io.setState(state);
  }
  public Command gameStateAwareLeds(Flywheel flywheel) {
    return run(() -> {
          if (DriverStation.isDisabled()) {
            setState(LedState.DISABLED);
            return;
          }
           if (flywheel.ready()) {
            setState(LedState.READY_TO_FIRE);
            return;
          }
          if (flywheel.isPreSpunUp()) {
            setState(LedState.SPINNING_UP);
            return;
          }
        //   if (RobotStates.pieceState == RobotStates.GamePieceState.HAS_PIECE) {
        //     setState(LedState.HAS_PIECE);
        //     return;
        //   }
        //    if (RobotStates.pieceState == RobotStates.GamePieceState.INTAKING) {
        //     setState(LedState.INTAKING);
        //     return;
        //   }

        //   if (RobotStates.location == RobotStates.FieldLocation.NEUTRAL_ZONE
        //       || RobotStates.location == RobotStates.FieldLocation.OPP_ALLINACE_ZONE) {
        //     setState(LedState.NEUTRAL_ZONE);
        //     return;
        //   }

          // Default
          setState(LedState.IDLE);
        })
        .ignoringDisable(true)
        .withName("Led.gameStateAwareLeds");
  }

  

  /** Force a specific state for the duration the command is scheduled. */
  public Command forceState(LedState state) {
    return runEnd(() -> setState(state), () -> setState(LedState.IDLE))
        .withName("Led.forceState(" + state.name() + ")");
  }

  public Command idle() {
    return forceState(LedState.IDLE);
  }

  public Command haspiece() {
    return forceState(LedState.HAS_PIECE);
  }

  public Command neutralZone() {
    return forceState(LedState.NEUTRAL_ZONE);
  }
}