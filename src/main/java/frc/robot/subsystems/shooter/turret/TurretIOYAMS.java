package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretIOYAMS implements TurretIO {
  private final TalonFX turretMotor;
  private final CANcoder encoder17;
  private final CANcoder encoder18;

  private final EasyCRT easyCrtSolver;

  // Inputs
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Voltage> turretVolts;
  private final StatusSignal<Current> turretCurrent;
  private final StatusSignal<Angle> enc17AbsPos;
  private final StatusSignal<Angle> enc18AbsPos;

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final PositionVoltage trackingRequest = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private boolean hasSeeded = false;

  public TurretIOYAMS() {
    CANBus rio = CANBus.roboRIO();
    turretMotor = new TalonFX(TurretConstants.kTurretMotorId, rio);
    encoder17 = new CANcoder(TurretConstants.kEncoder17Id, rio);
    encoder18 = new CANcoder(TurretConstants.kEncoder18Id, rio);

    CANcoderConfiguration encConfig = new CANcoderConfiguration();
    encoder17.getConfigurator().apply(encConfig);
    encoder18.getConfigurator().apply(encConfig);

    encConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder17.getConfigurator().apply(encConfig);
    encoder18.getConfigurator().apply(encConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    turretMotor.getConfigurator().apply(motorConfig);

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

    EasyCRTConfig easyCrtConfig =
        new EasyCRTConfig(
                () -> Rotations.of(enc17AbsPos.getValueAsDouble()),
                () -> Rotations.of(enc18AbsPos.getValueAsDouble()))
            .withAbsoluteEncoder1Gearing(
                TurretConstants.kTurretGearTeeth, TurretConstants.kEncoder1Teeth)
            .withAbsoluteEncoder2Gearing(
                TurretConstants.kTurretGearTeeth, TurretConstants.kEncoder2Teeth)
            .withMechanismRange(
                Rotations.of(TurretConstants.kMinAngle / 360.0),
                Rotations.of(TurretConstants.kMaxAngle / 360.0))
            .withAbsoluteEncoderOffsets(
                Rotations.of(TurretConstants.kEncoder17ZeroOffset),
                Rotations.of(TurretConstants.kEncoder18ZeroOffset))
            .withMatchTolerance(Rotations.of(0.06))
            .withAbsoluteEncoderInversions(
                TurretConstants.kEncoder17Inverted, TurretConstants.kEncoder18Inverted);

    easyCrtSolver = new EasyCRT(easyCrtConfig);
  }

  @Override
  public boolean encodersGood() {
    return BaseStatusSignal.isAllGood(enc17AbsPos, enc18AbsPos);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretPosition, turretVelocity, turretVolts, turretCurrent, enc17AbsPos, enc18AbsPos);

    inputs.turretPositionDeg = turretPosition.getValue().in(Degrees);
    inputs.turretVelocityRPM = turretVelocity.getValueAsDouble() * 60.0;

    inputs.turretAppliedVolts = turretVolts.getValueAsDouble();
    inputs.turretCurrentAmps = turretCurrent.getValueAsDouble();

    inputs.encoder17Rotations = enc17AbsPos.getValue().in(Rotations);
    inputs.encoder18Rotations = enc18AbsPos.getValue().in(Rotations);

    inputs.calculatedAbsPositionRot =
        easyCrtSolver.getAngleOptional().map(a -> a.in(Rotations)).orElse(0.0);
  }

  @Override
  public boolean atTarget(double angle) {
    double currentPosDeg = Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    double currentVelRpm = turretMotor.getVelocity().getValueAsDouble() * 60.0;

    boolean atPosition = Math.abs(angle - currentPosDeg) < TurretConstants.positionTolerance;
    boolean notMoving = Math.abs(currentVelRpm) < TurretConstants.velocityTolerance;

    return atPosition && notMoving;
  }

  @Override
  public void moveTo(double degrees) {
    turretMotor.setControl(trackingRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void setTurretVoltage(double volts) {
    turretMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    setTurretVoltage(0.0);
  }

  @Override
  public void seedTurretPosition() {

    easyCrtSolver
        .getAngleOptional()
        .ifPresent(
            mechAngle -> {
              StatusCode status = turretMotor.setPosition(mechAngle.in(Rotations), 0.05);
              if (status.isOK()) {
                hasSeeded = true;
                System.out.println(
                    "[Turret] YAMS EasyCRT Successfully seeded absolute position: "
                        + mechAngle.in(Rotations)
                        + " rotations.");
              } else {
                System.out.println("[Turret] Failed to seed turret motor: " + status.getName());
              }

              hasSeeded = true;
            });
  }
}
