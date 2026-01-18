package frc.robot.subsystems.lights;

import java.util.LinkedList;
import java.util.Objects;
import java.util.Queue;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {

    private static Queue<ControlRequest> requestQueue;
    private static int stripLength;

    private CANdle candle;

    public Lighting(int candleId, int numLeds) {
        Lighting.requestQueue = new LinkedList<>();
        candle = new CANdle(candleId);
        stripLength = numLeds + 8;

        // Clear all old animations from memory
        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }

        this.setDefaultCommand(pollForRequests());
    }

    private static void instantiationCheck() {
        Objects.requireNonNull(Lighting.requestQueue, () -> "Error: Lighting Subsystem not created. Create it in RobotContainer before using this method.");
    }

    /**
     * Use this function from any subsystem, to create lighting requests.
     *  The request will be processed during the next event loop cycle.
     * 
     * @param request A CANdle {@code ControlRequest}.
     */
    public static void request(ControlRequest request) {
        Lighting.instantiationCheck();
        Lighting.requestQueue.add(request);
    }

    /**
     * Clear the lighting requests queue and turn off the entire strip.
     */
    public static void clearAll() {
        Lighting.clearSegment(8, stripLength);
    }

    /**
     * Clear the lighting requests queue and turn off a segment of the strip.
     *  Useful when a subsystem is transitioning states & no longer needs to indicate anything.
     * 
     * @param start the start of the LED segment.
     * @param end the end of the LED segment.
     */
    public static void clearSegment(int start, int end) {
        Lighting.instantiationCheck();
        Lighting.requestQueue.clear();
        Lighting.requestQueue.add(
            new SolidColor(start, end).withColor(new RGBWColor(0, 0, 0, 0))
        );
    }

    /**
     * Run lighting requests once they are received.
     * 
     * @return A Command, to use as a default command for this Subsystem.
     */
    private Command pollForRequests() {
        return this.run(() -> {
            for(ControlRequest request : Lighting.requestQueue)
                candle.setControl(request);

            Lighting.requestQueue.clear();
        }).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        
    }
}
