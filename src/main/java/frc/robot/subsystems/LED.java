package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    public final AddressableLED m_led;
    public final AddressableLEDBuffer m_ledBuffer;

    private final LEDPattern m_red = LEDPattern.gradient(GradientType.kContinuous, new Color("#ff0000"),
            new Color("#220000"));
    private final LEDPattern m_blue = LEDPattern.gradient(GradientType.kContinuous, new Color("#0000ff"),
            new Color("#000022"));
    private final LEDPattern m_green = LEDPattern.solid(new Color("#00ff00"));
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 30.0);

    // of 1 meter per second.
    private final LEDPattern m_scrollingRed = m_red.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    private final LEDPattern m_scrollingBlue = m_blue.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    private LEDPattern m_defaultPattern;
    private LEDPattern m_curpattern;

    public LED() {
        m_led = new AddressableLED(6);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(80);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        m_curpattern = m_scrollingRainbow;
    }

    public Command setGreen() {
        return runOnce(() -> {
            m_curpattern = m_green;
        });
    }

    public Command setDefault() {
        return runOnce(() -> {
            m_curpattern = m_defaultPattern;
        });
    }

    public Command setRainbow() {
        return run(() -> {
            m_curpattern = m_scrollingRainbow;
        });
    }

    public void periodic() {
        if (m_defaultPattern == null && DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                m_defaultPattern = m_scrollingRed;
            } else {
                m_defaultPattern = m_scrollingBlue;
            }
        }

        m_curpattern.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }
}
