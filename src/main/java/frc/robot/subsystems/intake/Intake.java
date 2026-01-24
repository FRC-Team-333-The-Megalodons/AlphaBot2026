package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
    public Command runIntakeCommand() {
    return runEnd(() -> this.run(true), this::stop);
    }

    public Command runOuttakeCommand() {
    return runEnd(() -> this.run(false), this::stop);
    }

    public void run(boolean forward) {
        io.setVoltage(forward ? IntakeConstants.INTAKE_VOLTS : -IntakeConstants.INTAKE_VOLTS);
    }

    public void stop() {
        io.setVoltage(0.0);
    }
}