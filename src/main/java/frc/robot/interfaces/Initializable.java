package frc.robot.interfaces;

public interface Initializable {
    
    /**
     * Any Subsystems, which require DriverStation connectivity, data, or a start-up delay, should implement this function.
     * 
     * This function should then be called in <code>Robot.driverStationConnected</code>.
     */
    public void seed();
}
