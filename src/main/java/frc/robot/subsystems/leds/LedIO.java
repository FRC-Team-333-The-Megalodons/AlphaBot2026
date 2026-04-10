package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {

  @AutoLog
  public static class LedIOInputs {
    public String currentState = LedState.IDLE.name();
  }

  public default void updateInputs(LedIOInputs inputs) {}

  public default void setState(LedState state) {}

  public default boolean anyCameraSeesTag() { return false; }
}
