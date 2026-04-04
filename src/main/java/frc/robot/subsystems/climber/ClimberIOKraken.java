package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOKraken implements ClimberIO {

  private final CANBus rio = CANBus.roboRIO();
  private final TalonFX motor = new TalonFX(ClimberConstants.MOTOR_ID, rio);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Current> supplyCurrentAmps;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  private boolean hasZeroed = false;

  public ClimberIOKraken() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    config.Slot0.kP = ClimberConstants.kP;
    config.Slot0.kI = ClimberConstants.kI;
    config.Slot0.kD = ClimberConstants.kD;
    config.Slot0.kS = ClimberConstants.kS;
    config.Slot0.kV = ClimberConstants.kV;
    config.Slot0.kA = ClimberConstants.kA;
    config.Slot0.kG = ClimberConstants.kG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.kCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ClimberConstants.kAcceleration;
    config.MotionMagic.MotionMagicJerk = ClimberConstants.kJerk;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // NO soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    config.CurrentLimits.StatorCurrentLimit = 70.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    supplyCurrentAmps = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, currentAmps, supplyCurrentAmps);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, currentAmps, supplyCurrentAmps);

    inputs.positionRot = position.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.supplyAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.hasZeroed = hasZeroed;
  }

  @Override
  public void moveTo(double positionRot) {
    motor.setControl(motionMagicRequest.withPosition(positionRot));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setDutyCycle(double percent) {
    motor.setControl(dutyCycleRequest.withOutput(percent));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void zeroPosition() {
    motor.setPosition(0.0);
    hasZeroed = true;
    System.out.println("[Climber] Encoder zeroed.");
  }

  @Override
  public boolean atTarget(double positionRot) {
    double currentPos = position.getValueAsDouble();
    double currentVel = velocity.getValueAsDouble();

    boolean atPosition =
        Math.abs(positionRot - currentPos) < ClimberConstants.POSITION_TOLERANCE_ROT;
    boolean notMoving = Math.abs(currentVel) < ClimberConstants.VELOCITY_TOLERANCE_RPS;

    return atPosition && notMoving;
  }
}
