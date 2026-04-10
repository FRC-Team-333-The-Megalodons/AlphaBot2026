package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SpindexerIOKraken implements SpindexerIO {
  private final TalonFX motor;

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Temperature> tempSignal;

  private final MotionMagicVelocityVoltage velocityRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public SpindexerIOKraken() {
    motor = new TalonFX(SpindexerConstants.MOTOR_ID, "canivore");

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = SpindexerConstants.GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kS = SpindexerConstants.kS;
    config.Slot0.kP = SpindexerConstants.kP;
    config.Slot0.kA = SpindexerConstants.kA;
    config.Slot0.kV = SpindexerConstants.kV;

    config.MotionMagic.MotionMagicAcceleration = SpindexerConstants.MAX_ACCEL;
    config.MotionMagic.MotionMagicJerk = SpindexerConstants.MAX_JERK;

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    tempSignal = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocitySignal, voltageSignal, currentSignal, supplyCurrentSignal, tempSignal);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocitySignal, voltageSignal, currentSignal, supplyCurrentSignal, tempSignal);

    inputs.velocityRps = velocitySignal.getValueAsDouble();
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.statorAmps = currentSignal.getValueAsDouble();
    inputs.supplyAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.tempCelsius = tempSignal.getValueAsDouble();
  }

  @Override
  public void moveTo(double rpm) {
    double rps = rpm / 60.0;
    motor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public boolean atTarget(double rpm) {
    double currentRPM = velocitySignal.getValueAsDouble() * 60.0;
    return Math.abs(Math.abs(currentRPM) - Math.abs(rpm))
        < SpindexerConstants.VELOCITY_TOLERANCE_RPM;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
