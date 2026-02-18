package frc.robot.subsystems.transfer;

public class TransferIOKrakenSim implements TransferIO {
  private double volts = 0.0;

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.appliedVolts = volts;
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }
}
