// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import dev.doglog.DogLog;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.constants.Superstructure.WristConstants;
import frc.robot.constants.Superstructure.CANIds;
import frc.robot.constants.Superstructure.Configs;
import frc.robot.constants.Superstructure.Mechanisms;
import frc.robot.constants.Superstructure.PhysicalRobotConstants;
import frc.robot.util.Tracer;

@Logged
public class Wrist extends SubsystemBase {
  public enum WristState {
    LevelNormal(40),
    Level4(55),
    Handoff(0),
    Safe(45);

    public double angle;

    WristState(double angle) {
      this.angle = angle;
    }

  }

  public final Trigger atMin = new Trigger(() -> getAngle().lte(WristConstants.kMinAngle.plus(Degrees.of(7))));
  public final Trigger atMax = new Trigger(() -> getAngle().gte(WristConstants.kMaxAngle.minus(Degrees.of(5))));

  private SparkMax wristMotor = new SparkMax(
      CANIds.kWristMotorCanId,
      MotorType.kBrushless);
  private final ProfiledPIDController m_wristPIDController = new ProfiledPIDController(
      WristConstants.kWristkP,
      WristConstants.kWristkI,
      WristConstants.kWristkD,
      new Constraints(
          WristConstants.kWristMaxVelocityRPM,
          WristConstants.kWristMaxAccelerationRPMperSecond));
  private final ArmFeedforward m_wristFeedforward = new ArmFeedforward(
      WristConstants.kWristkS,
      WristConstants.kWristkG,
      WristConstants.kWristkV,
      WristConstants.kWristkA);

  private RelativeEncoder wristEncoder = wristMotor.getEncoder();
  private AbsoluteEncoder wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();
  private WristState wristCurrentTarget = WristState.Handoff;
  private Notifier simNotifier = null;

  private DCMotor wristMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim wristMotorSim;
  private SparkAbsoluteEncoderSim absoluteEncoderSim;
  private final SingleJointedArmSim m_wristSim = new SingleJointedArmSim(
      wristMotorModel,
      PhysicalRobotConstants.kWristReduction,
      SingleJointedArmSim.estimateMOI(
          PhysicalRobotConstants.kWristLength.in(Meters),
          PhysicalRobotConstants.kWristMass.in(Kilograms)),
      PhysicalRobotConstants.kWristLength.in(Meters),
      PhysicalRobotConstants.kMinAngleRads,
      PhysicalRobotConstants.kMaxAngleRads,
      false,
      PhysicalRobotConstants.kMinAngleRads,
      0.0,
      0.0);

  // Mechanism2d setup for subsystem
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutAngle m_angle = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutAngularVelocity m_velocity = RPM.mutable(0);
  // SysID Routine
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(6), Seconds.of(10)),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motor(s).
          wristMotor::setVoltage,
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the shooter motor.
            log.motor("arm")
                .voltage(
                    m_appliedVoltage.mut_replace(wristMotor.getAppliedOutput() *
                        RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(getArmActualPosition(), Rotations))
                .angularVelocity(m_velocity.mut_replace(wristAbsoluteEncoder.getVelocity(), RPM));
          },
          this));

  public Wrist() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    wristMotor.configure(
        Configs.WristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize simulation values
    wristMotorSim = new SparkMaxSim(wristMotor, wristMotorModel);
    absoluteEncoderSim = new SparkAbsoluteEncoderSim(wristMotor);
    if (Robot.isSimulation()) {
      simNotifier = new Notifier(() -> {
        updateSimState();
      });
      simNotifier.startPeriodic(0.005);
      absoluteEncoderSim.setPosition(0);
    }
    wristEncoder.setPosition(wristAbsoluteEncoder.getPosition());

    m_wristPIDController.enableContinuousInput(0, 1);
  }

  public void updateSimState() {
    DogLog.log("Wrist/vbatt", RobotController.getBatteryVoltage());
    m_wristSim.setInputVoltage(
        wristMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    m_wristSim.update(0.0050);

    wristMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_wristSim.getVelocityRadPerSec()) * PhysicalRobotConstants.kWristReduction,
        RobotController.getBatteryVoltage(),
        0.005);
    absoluteEncoderSim.iterate(Units.radiansPerSecondToRotationsPerMinute(
        m_wristSim.getVelocityRadPerSec()), 0.005);

  }

  private void moveToSetpoint() {
    double pidOutput = m_wristPIDController.calculate(
        getArmActualPosition(),
        Units.degreesToRotations(wristCurrentTarget.angle));
    State setpointState = m_wristPIDController.getSetpoint();

    double ffOut = m_wristFeedforward.calculate(setpointState.position + 0.36111111, setpointState.velocity);
    DogLog.log("Wrist/pidOutput", pidOutput);
    DogLog.log("Wrist/FFOut", ffOut);
    wristMotor.setVoltage(
        pidOutput +
            ffOut);
  }

  /**
   * Do not use unless you understand that it exits immediately, NOT
   * after it reaches setpoint
   * 
   * @param setpoint
   * @return
   * @deprecated
   */
  public Command setSetpointCommand(WristState wristSetpoint) {
    return this.runOnce(() -> {
      this.wristCurrentTarget = wristSetpoint;
    });
  }

  @Override
  public void periodic() {
    Tracer.startTrace("Wrist Periodic");
    Tracer.traceFunc("Wrist moveToSetpoint", this::moveToSetpoint);

    Mechanisms.m_wristMech2d.setAngle(
        180 -
            (Units.radiansToDegrees(PhysicalRobotConstants.kMinAngleRads) +
                Units.rotationsToDegrees(
                    getArmActualPosition()))
            -
            90);
    Tracer.endTrace();
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_wristSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
  }

  public double getArmActualPosition() {
    var realPos = wristAbsoluteEncoder.getPosition();
    return realPos > 0.99 ? 0 : realPos;
  }

  public Trigger atAngle(double angle, double tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(angle,
          Units.rotationsToDegrees(getArmActualPosition()), tolerance);
    });
  }

  public Trigger atSetpoint = new Trigger(() -> {
    return MathUtil.isNear(wristCurrentTarget.angle,
        Units.rotationsToDegrees(getArmActualPosition()), 8);
  });

  public Trigger atSetpoint(WristState setpoint) {
    return new Trigger(() -> {
      return MathUtil.isNear(setpoint.angle,
          Units.rotationsToDegrees(getArmActualPosition()), 8);
    });
  };

  public Angle getAngle() {
    return Rotations.of(getArmActualPosition());
  }

  public Command runSysIdRoutine() {
    return m_sysIdRoutine.dynamic(Direction.kForward).until(atMax)
        .andThen(Commands.waitSeconds(1))

        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))

        .andThen(Commands.waitSeconds(1))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(Commands.waitSeconds(1))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin)).andThen(Commands.print("done"));
  }

  public Command wristToPosition(WristState state) {
    return Commands.sequence(setSetpointCommand(state), Commands.waitUntil(atSetpoint));
  }

}
