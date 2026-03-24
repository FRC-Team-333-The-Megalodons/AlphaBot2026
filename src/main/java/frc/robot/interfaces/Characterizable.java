package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Any Subsystem with a SysId-characterizable mechanism, or a tunable PID controller, should implement this interface
 */
public interface Characterizable {
  
  public Command characterize();
  
  private Command pause() {
    return Commands.waitSeconds(1.0);
  }

  public default Command runSysIdSequence(SysIdRoutine routine) {
    return Commands.sequence(
      pause(),
      routine.quasistatic(Direction.kForward),
      pause(),
      routine.quasistatic(Direction.kReverse),
      pause(),
      routine.dynamic(Direction.kForward),
      pause(),
      routine.dynamic(Direction.kReverse),
      pause()
    );
  }
}
