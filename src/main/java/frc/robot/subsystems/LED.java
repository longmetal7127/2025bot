package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LED extends SubsystemBase {
    public final AddressableLED m_led;
    public final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(35);
    AddressableLEDBufferView m_right = m_ledBuffer.createView(0, 11);
    AddressableLEDBufferView m_left = m_ledBuffer.createView(12, 22);
    AddressableLEDBufferView m_middle = m_ledBuffer.createView(23, 34);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 30.0);
    private LEDState m_curpattern = getRandomFlag();

    public enum LEDState {
        RED_SCROLLING(LEDPattern.gradient(GradientType.kContinuous, new Color("#ff0000"),
                new Color("#220000")).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        BLUE_SCROLLING(LEDPattern.gradient(GradientType.kContinuous, new Color("#0000ff"),
                new Color("#000022")).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        RAINBOW_SCROLLER(LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        FAST_RAINBOW_SCROLLER(LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(2), kLedSpacing)),
        LESBIAN_FLAG(LEDPattern.steps(Map.of(
                0.0, new Color("#D62900"),
                0.1667, new Color("#FF9B56"),
                0.3333, new Color("#FFFFFF"),
                0.5, new Color("#D463A6"),
                0.6667, new Color("#D463A6"),
                0.8333, new Color("#A50062"))).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        TRANS_FLAG(LEDPattern.steps(Map.of(
                0.0, new Color("#75D7EE"),
                0.25, new Color("#FFAFC8"),
                0.5, new Color("#FFFFFF"),
                0.75, new Color("#FFAFC8"))).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        BI_FLAG(LEDPattern.steps(Map.of(
                0.0, new Color("#D60370"),
                0.3333, new Color("#9B4F96"),
                0.6667, new Color("#0038A8"))).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        PAN_FLAG(LEDPattern.steps(Map.of(
                0.0, new Color("#F72088"),
                0.33, new Color("#FFD800"),
                0.67, new Color("#22B1FF"))).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),

        GENDERFLUID_FLAG(LEDPattern.steps(Map.of(
                0.0, new Color("#8D4A9F"),
                0.2, new Color("#DC75B4"),
                0.4, new Color("#FFFFFF"),
                0.6, new Color("#424391"),
                0.8, new Color("#000625"))).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),

        ROMANIAN_FLAG(LEDPattern.steps(Map.of(
                0.0, new Color("#002B7F"), 0.33, new Color("#FCD116"), 0.67, new Color("#CE1126")))
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        AROACE_FLAG(LEDPattern.steps(Map.of(
                0.0, new Color("#E38D00"),
                0.2, new Color("#EDCE01"),
                0.4, new Color("#FFFFFF"),
                0.6, new Color("#62B0DD"),
                0.8, new Color("#1B3556"))).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),
        GAY_MEN(LEDPattern.steps(Map.of(
                0.0, new Color("#09896C"),
                0.1429, new Color("#24C8A5"),
                0.2857, new Color("#94E1BC"),
                0.4286, new Color("#FFFFFF"),
                0.5714, new Color("#7CACE2"),
                0.7143, new Color("#5049CB"),
                0.8571, new Color("#3E1A78"))).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing)),


        GREEN(LEDPattern.solid(new Color("#00ff00")));

        public LEDPattern pattern;

        LEDState(LEDPattern pattern) {
            this.pattern = pattern;
        }
    }

    public LED() {
        m_led = new AddressableLED(6);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public Command setGreen() {
        return runOnce(() -> {
            m_curpattern = LEDState.GREEN;
        });
    }

    public Command setDefault() {
        return runOnce(() -> {
            m_curpattern = Robot.isOnRed() ? LEDState.RED_SCROLLING : LEDState.BLUE_SCROLLING;
        });
    }

    public LEDState getRandomFlag() {
        LEDState[] prideFlags = {
                LEDState.LESBIAN_FLAG,
                LEDState.BI_FLAG,
                LEDState.TRANS_FLAG,
                LEDState.PAN_FLAG,
                LEDState.GENDERFLUID_FLAG,
                LEDState.ROMANIAN_FLAG,
                LEDState.AROACE_FLAG,
                LEDState.GAY_MEN
        };
        int randomIndex = (int) (Math.random() * prideFlags.length);
        return prideFlags[randomIndex];
    }

    public Command setRainbow() {
        // pick random of pride flags
        return runOnce(() -> {

            m_curpattern = getRandomFlag();

        });
    }

    public Command setFastRainbow() {
        return runOnce(() -> {
            m_curpattern = LEDState.FAST_RAINBOW_SCROLLER;
        });
    }

    public void periodic() {
        m_curpattern.pattern.applyTo(m_ledBuffer);
        //m_curpattern.pattern.applyTo(m_right);
        //m_curpattern.pattern.applyTo(m_middle);

        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }
}