package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurretIOLimelight implements TurretIO {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public TurretIOLimelight() {
    // Configure Limelight or Motor here if needed
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
    if (botpose.length >= 6) {
      double yawDegrees = botpose[5];
      inputs.positionRad = Units.degreesToRadians(yawDegrees);
      inputs.limelightImuYaw = inputs.positionRad;
    }

    inputs.limelightHasTarget = table.getEntry("tv").getDouble(0) == 1;
    inputs.limelightTx = table.getEntry("tx").getDouble(0);
  }

  @Override
  public void setVoltage(double volts) {}
}
