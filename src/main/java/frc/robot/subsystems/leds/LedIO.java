package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {

  @AutoLog
  public static class LedIOInputs {
    public String currentState = LedState.IDLE.name();
    public boolean camera0SeesTag = false;
    public boolean camera1SeesTag = false;
  }

  public default void updateInputs(LedIOInputs inputs) {}

  public default void setState(LedState state) {}

  public default void setVisionState(boolean camera0SeesTag, boolean camera1SeesTag) {}
}
