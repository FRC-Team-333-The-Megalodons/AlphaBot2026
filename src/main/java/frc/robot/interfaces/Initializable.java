package frc.robot.interfaces;

/**
 * Any Subsystems, which require DriverStation connectivity, data, or a start-up delay, should
 * implement this function.
 */
public interface Initializable {

  /**
   * Call this function from <code>Robot.driverStationConnected</code>, to seed the subsystem with
   * DriverStation data.
   */
  public void seed();
}
