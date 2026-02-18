package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOKraken implements TurretIO {
  private final TalonFX turretMotor;
  private final CANcoder encoder17;
  private final CANcoder encoder18;

  // Inputs
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Voltage> turretVolts;
  private final StatusSignal<Current> turretCurrent;
  private final StatusSignal<Angle> enc17AbsPos;
  private final StatusSignal<Angle> enc18AbsPos;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public TurretIOKraken() {
    CANBus rio = CANBus.roboRIO();
    turretMotor = new TalonFX(TurretConstants.kTurretMotorId, rio);
    encoder17 = new CANcoder(TurretConstants.kEncoder17Id, rio);
    encoder18 = new CANcoder(TurretConstants.kEncoder18Id, rio);

    CANcoderConfiguration encConfig = new CANcoderConfiguration();
    encConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoder17.getConfigurator().apply(encConfig);
    encoder18.getConfigurator().apply(encConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Feedback.SensorToMechanismRatio = TurretConstants.kMotorToTurretRatio;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.Slot0.kP = TurretConstants.kP;
    motorConfig.Slot0.kI = TurretConstants.kI;
    motorConfig.Slot0.kD = TurretConstants.kD;
    motorConfig.Slot0.kS = TurretConstants.kS;
    motorConfig.Slot0.kV = TurretConstants.kV;
    motorConfig.Slot0.kA = TurretConstants.kA;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.kAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = TurretConstants.kJerk;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(TurretConstants.kMaxAngle);
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(TurretConstants.kMinAngle);
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    turretMotor.getConfigurator().apply(motorConfig);

    turretPosition = turretMotor.getPosition();
    turretVelocity = turretMotor.getVelocity();
    turretVolts = turretMotor.getMotorVoltage();
    turretCurrent = turretMotor.getStatorCurrent();
    enc17AbsPos = encoder17.getAbsolutePosition();
    enc18AbsPos = encoder18.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, turretPosition, turretVelocity, turretVolts, turretCurrent, enc17AbsPos, enc18AbsPos);

    // Seed Absolute Position once the Robot Boots
    seedTurretPosition();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretPosition, turretVelocity, turretVolts, turretCurrent, enc17AbsPos, enc18AbsPos);

    inputs.connected = BaseStatusSignal.isAllGood(turretPosition, enc17AbsPos);
    inputs.turretPositionRad = Units.rotationsToRadians(turretPosition.getValueAsDouble());
    inputs.turretVelocityRadPerSec = Units.rotationsToRadians(turretVelocity.getValueAsDouble());
    inputs.turretAppliedVolts = turretVolts.getValueAsDouble();
    inputs.turretCurrentAmps = turretCurrent.getValueAsDouble();

    inputs.encoder17Rotations = enc17AbsPos.getValueAsDouble();
    inputs.encoder18Rotations = enc18AbsPos.getValueAsDouble();
    inputs.calculatedAbsPositionRot =
        calculateAbsolutePosition(inputs.encoder17Rotations, inputs.encoder18Rotations);
  }

  @Override
  public void setTurretPosition(Rotation2d position) {
    turretMotor.setControl(positionRequest.withPosition(position.getRotations()));
  }

  @Override
  public void setTurretVoltage(double volts) {
    turretMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    turretMotor.setControl(voltageRequest.withOutput(0));
  }

  /** Calculates the absolute turret position using the CRT */
  private double calculateAbsolutePosition(double p17, double p18) {
    double n1 = 17.0;
    double n2 = 18.0;
    double N = 105.0;

    double delta = (n2 * p18) - (n1 * p17);
    long k_diff = Math.round(delta);

    double rawTurretRot = (-k_diff + p17) * (n1 / N);

    double offsetPosition = rawTurretRot - TurretConstants.kTurretZeroOffset;

    double period = (n1 * n2) / N; // ~2.914

    while (offsetPosition > period / 2.0) offsetPosition -= period;
    while (offsetPosition < -period / 2.0) offsetPosition += period;

    return offsetPosition;
  }

  private void seedTurretPosition() {
    BaseStatusSignal.refreshAll(enc17AbsPos, enc18AbsPos);
    double absPos =
        calculateAbsolutePosition(enc17AbsPos.getValueAsDouble(), enc18AbsPos.getValueAsDouble());
    turretMotor.setPosition(absPos);
  }
}
