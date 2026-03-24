package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private boolean hasSeeded = false;

  public TurretIOKraken() {
    CANBus rio = CANBus.roboRIO();
    turretMotor = new TalonFX(TurretConstants.kTurretMotorId, rio);
    encoder17 = new CANcoder(TurretConstants.kEncoder17Id, rio);
    encoder18 = new CANcoder(TurretConstants.kEncoder18Id, rio);

    CANcoderConfiguration encConfig = new CANcoderConfiguration();
    encConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
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

    // motorConfig.CurrentLimits.StatorCurrentLimit = 50.0;
    // motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // motorConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
    // motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turretMotor.getConfigurator().apply(motorConfig);

    turretPosition = turretMotor.getPosition();
    turretVelocity = turretMotor.getVelocity();
    turretVolts = turretMotor.getMotorVoltage();
    turretCurrent = turretMotor.getStatorCurrent();
    enc17AbsPos = encoder17.getAbsolutePosition();
    enc18AbsPos = encoder18.getAbsolutePosition();
    statorCurrent = turretMotor.getStatorCurrent();
    supplyCurrent = turretMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        turretPosition,
        turretVelocity,
        turretVolts,
        turretCurrent,
        enc17AbsPos,
        enc18AbsPos,
        statorCurrent,
        supplyCurrent);
  }

  public boolean areEncoderValuesSane(double enc17, double enc18) {
    // In the real world, neither encoder should ever give a true absolute zero value;
    // If they do, you know it is B/S (i.e. its giving you a placeholder start value)
    return (enc17 != 0.0 && enc18 != 0.0);
  }

  public static long lastSeededTime = 0;
  public static final long turretCrtSeedDelay = 5000; // millseconds

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Seed Absolute Position once the Robot Boots
    BaseStatusSignal.refreshAll(
        turretPosition,
        turretVelocity,
        turretVolts,
        turretCurrent,
        enc17AbsPos,
        enc18AbsPos,
        supplyCurrent,
        statorCurrent);

    // if (!hasSeeded || turretMotor.hasResetOccurred()) {
    long now = System.currentTimeMillis();
    if (now - lastSeededTime > turretCrtSeedDelay) {
      if (BaseStatusSignal.isAllGood(enc17AbsPos, enc18AbsPos)
          && areEncoderValuesSane(enc17AbsPos.getValueAsDouble(), enc18AbsPos.getValueAsDouble())) {
        double absPos =
            calculateAbsolutePosition(
                enc17AbsPos.getValueAsDouble(), enc18AbsPos.getValueAsDouble());

        StatusCode motorStatus = turretMotor.setPosition(absPos, 0.01);
        if (motorStatus == StatusCode.OK) {
          hasSeeded = true;
          lastSeededTime = now;
          System.out.println("[Turret] Successfully seeded absolute position: " + absPos);
        }
      }
    }

    // inputs.connected = BaseStatusSignal.isAllGood(turretPosition, enc17AbsPos);
    inputs.turretPositionDeg = turretPosition.getValue().in(Degrees);
    inputs.turretVelocityRPM = turretVelocity.getValueAsDouble() * 60.0;
    inputs.turretAppliedVolts = turretVolts.getValueAsDouble();
    inputs.turretStatorAmps = turretCurrent.getValueAsDouble();
    inputs.turretSupplyAmps = supplyCurrent.getValueAsDouble();
    inputs.turretStatorAmps = statorCurrent.getValueAsDouble();

    inputs.encoder17Rotations = enc17AbsPos.getValueAsDouble();
    inputs.encoder18Rotations = enc18AbsPos.getValueAsDouble();
    inputs.calculatedAbsPositionRot =
        calculateAbsolutePosition(inputs.encoder17Rotations, inputs.encoder18Rotations);
  }

  @Override
  public void moveTo(double degrees) {
    turretMotor.setControl(positionRequest.withPosition(Units.degreesToRotations(degrees)));
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

  @Override
  public void seedTurretPosition() {
    hasSeeded = false;
  }
}
