package frc.robot.subsystems.leds;

public enum LedState {
  // No command active —> LEDs off.
  IDLE,

  // Intake is running —> robot is searching for a ball.
  INTAKING,

  // Ball is loaded and ready to shoot.
  HAS_PIECE,

  // Flywheel is spinning up, not at target RPM yet.
  SPINNING_UP,

  // Flywheel is at target RPM —> ready to fire.
  READY_TO_FIRE,

  // Robot is in the neutral zone —> warn drivers to consider passing.
  NEUTRAL_ZONE,

  // Robot is disabled.
  DISABLED,

  // The intake is spun up at a reasonable RPM (i.e. its not stuck)
  INTAKE_IS_AT_SPEED,

  // Shooter has at least one Apriltag
  SHOOTER_HAS_TAG,

  // Climber is UP
  CLIMBER_IS_UP
}
