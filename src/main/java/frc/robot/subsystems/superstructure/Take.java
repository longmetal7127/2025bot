package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Superstructure;
import frc.robot.constants.Superstructure.Configs;
import frc.robot.util.Tracer;

@Logged
public class Take extends SubsystemBase {
    private SparkMax takeMotor = new SparkMax(Superstructure.CANIds.kTakeMotorCanId, MotorType.kBrushless);
    public Trigger hasCoral = new Trigger(takeMotor.getForwardLimitSwitch()::isPressed);
    private LinearFilter coralFilter = LinearFilter.highPass(0.1, 0.02);
    public Trigger justGotCoral = new Trigger(() -> {
        return coralFilter.calculate((takeMotor.getOutputCurrent())) > 1;
    }).debounce(0.04);

    SysIdRoutine sysIdRoutine;
    private double setpoint = 0;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.2015, 0.0020986, 0);
    ProfiledPIDController controller = new ProfiledPIDController(
            0, 0, 0,
            new TrapezoidProfile.Constraints(4000, 10000));
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutAngle m_angle = Rotations.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutAngularVelocity m_velocity = RPM.mutable(0);

    public Take() {
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> takeMotor.setVoltage(voltage),
                        log -> {
                            log.motor("take")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(takeMotor.getAppliedOutput() *
                                                    RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(
                                            m_angle.mut_replace(takeMotor.getEncoder().getPosition(), Rotations))
                                    .angularVelocity(m_velocity.mut_replace(takeMotor.getEncoder().getVelocity(), RPM));
                        },
                        this));

        takeMotor.configure(
                Configs.takeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        takeMotor.getEncoder().getPosition();
        takeMotor.getEncoder().getVelocity();

    }

    public Command runTakeMotor() {
        return startEnd(() -> {
            setpoint = 1200;
        }, () -> {
            setpoint = 0;
        });
    }

    public Command runTakeMotorReverse() {
        return startEnd(() -> {
            setpoint = -1200;
        }, () -> {
            setpoint = 0;
        });
    }

    public Command runTakeMotorReverse(double speed) {
        return startEnd(() -> {
            setpoint = speed;
        }, () -> {
            setpoint = 0;
        });
    }

    public void periodic() {
        Tracer.startTrace("TakePeriodic");
        takeMotor.setVoltage(controller.calculate(takeMotor.getEncoder().getVelocity(), setpoint) +
                feedforward.calculate(setpoint));
        Tracer.endTrace();

    }

    public Command runSysIdRoutine() {
        return (sysIdRoutine.dynamic(Direction.kForward))
                .andThen(sysIdRoutine.dynamic(Direction.kReverse))
                .andThen(sysIdRoutine.quasistatic(Direction.kForward))
                .andThen(sysIdRoutine.quasistatic(Direction.kReverse))
                .andThen(Commands.print("DONE"));
    }

}
