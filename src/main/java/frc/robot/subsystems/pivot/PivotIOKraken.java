package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class PivotIOKraken implements PivotIO {
  CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(PivotConstants.MOTOR_ID, rio);

  public PivotIOKraken() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = PivotConstants.GEAR_RATIO;
    config.Slot0.kP = PivotConstants.kP;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setPosition(double rad) {
    motor.setControl(new PositionVoltage(Units.radiansToRotations(rad)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
