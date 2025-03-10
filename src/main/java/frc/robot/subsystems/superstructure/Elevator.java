// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.constants.Superstructure.CANIds;
import frc.robot.constants.Superstructure.Configs;
import frc.robot.constants.Superstructure.ElevatorConstants;
import frc.robot.constants.Superstructure.Mechanisms;
import frc.robot.constants.Superstructure.PhysicalRobotConstants;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.util.Tracer;

@Logged
public class Elevator extends SubsystemBase {
  public enum ElevatorState {
    Min(0),
    Level1(0),
    Level2(0.25),
    Level3(0.70),
    Level4(1.35),
    SourcePickup(0.2),
    Handoff(0);

    public double height;

    ElevatorState(double height) {
      this.height = height;
    }

  }

  private final ProfiledPIDController m_elevatorPIDController = new ProfiledPIDController(
      ElevatorConstants.kElevatorkP,
      ElevatorConstants.kElevatorkI,
      ElevatorConstants.kElevatorkD,
      new Constraints(
          ElevatorConstants.kElevatorMaxVelocity,
          ElevatorConstants.kElevatorMaxAcceleration));

  private final ElevatorFeedforward m_ElevatorFeedforwardStage1 = new ElevatorFeedforward(
      ElevatorConstants.kElevatorkS,
      ElevatorConstants.kElevatorkGStage1,
      ElevatorConstants.kElevatorkV,
      ElevatorConstants.kElevatorkA);
  private final ElevatorFeedforward m_ElevatorFeedforwardStage2 = new ElevatorFeedforward(
      ElevatorConstants.kElevatorkS + .1,
      ElevatorConstants.kElevatorkGStage2,
      ElevatorConstants.kElevatorkV,
      ElevatorConstants.kElevatorkA);

  private SparkMax elevatorMotor = new SparkMax(
      CANIds.kElevatorMotorCanId,
      MotorType.kBrushless);
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  private SparkMax elevatorFollowerMotor = new SparkMax(
      CANIds.kElevatorMotorFollowerCanId,
      MotorType.kBrushless);

  private ElevatorState elevatorCurrentTarget = ElevatorState.Min;
  private Notifier simNotifier = null;

  private DCMotor elevatorMotorModel = DCMotor.getNEO(2);
  private SparkMaxSim elevatorMotorSim;
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      elevatorMotorModel,
      PhysicalRobotConstants.kElevatorGearing,
      PhysicalRobotConstants.kCarriageMass,
      PhysicalRobotConstants.kElevatorDrumRadius,
      PhysicalRobotConstants.kMinElevatorCarriageHeightMeters,
      PhysicalRobotConstants.kMaxElevatorStage1HeightMeters,
      true,
      0,
      0.0,
      0.0);
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutAngle m_rotations = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(4.5), Seconds.of(10)),
      new SysIdRoutine.Mechanism(
          (voltage) -> elevatorMotor.setVoltage(voltage),

          log -> {
            // Record a frame for the shooter motor.
            log.motor("elevator")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getLinearPositionMeters(),
                    Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
                    MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
          },
          this));

  public final Trigger atMin = new Trigger(() -> getLinearPosition().isNear(Meters.of(0),
      Inches.of(5)));
  public final Trigger atMax = new Trigger(() -> getLinearPosition().isNear(Meters.of(0.668),
      Inches.of(3)));

  public Elevator() {
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
    elevatorMotor.configure(
        Configs.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorFollowerMotor.configure(
        Configs.elevatorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putData("elevator.mech2d", Mechanisms.m_mech2d);

    elevatorEncoder.setPosition(0);
    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorMotorModel);
    Mechanisms.m_elevatorCarriageMech2d.setColor(new Color8Bit("#ff0000"));
    Mechanisms.m_elevatorStage1Mech2d.setColor(new Color8Bit("#00ff00"));
    if (Robot.isSimulation()) {
      simNotifier = new Notifier(() -> {
        updateSimState();
      });
      simNotifier.startPeriodic(0.005);
    }
  }

  public void updateSimState() {
    m_elevatorSim.setInput(
        elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    m_elevatorSim.update(0.0050);

    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond() /
            (PhysicalRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI)) *
            PhysicalRobotConstants.kElevatorGearing) *
            60.0,
        RobotController.getBatteryVoltage(),
        0.005);

  }

  private void moveToSetpoint() {
    var feedforward = convertRotationsToMeters(elevatorEncoder.getPosition()) >= 0.70 ? m_ElevatorFeedforwardStage2 : m_ElevatorFeedforwardStage1;

    elevatorMotor.setVoltage(
        m_elevatorPIDController.calculate(
          convertRotationsToMeters(elevatorEncoder.getPosition()),
            elevatorCurrentTarget.height) +
            feedforward.calculate(
                m_elevatorPIDController.getSetpoint().velocity));

  }

  /**
   * Do not use unless you understand that it exits immediately, NOT
   * after it reaches setpoint
   * 
   * @param setpoint
   * @return
   * @deprecated
   */
  public Command setSetpointCommand(ElevatorState setpoint) {
    return Commands.sequence(
        this.runOnce(() -> {
          this.elevatorCurrentTarget = setpoint;
          m_elevatorPIDController.reset(getLinearPositionMeters());
        }));
  }

  @Override
  public void periodic() {
    Tracer.startTrace("ElevatorPeriodic");
    moveToSetpoint();
    /*double height = (elevatorEncoder.getPosition() /
        PhysicalRobotConstants.kElevatorGearing) *
        (PhysicalRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI);
    double carriageHeight = Math.min(
        PhysicalRobotConstants.kCarriageTravelHeightMeters,
        height);
    double stage1Height = (carriageHeight == PhysicalRobotConstants.kCarriageTravelHeightMeters)
        ? height - PhysicalRobotConstants.kCarriageTravelHeightMeters
        : 0;
    Mechanisms.m_elevatorCarriageMech2d.setLength(
        PhysicalRobotConstants.kMinElevatorCarriageHeightMeters + carriageHeight);
    Mechanisms.m_elevatorStage1Mech2d.setLength(
        PhysicalRobotConstants.kMinElevatorStage1HeightMeters + stage1Height);*/

    Tracer.endTrace();
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)

  }

  public Pose3d[] getMechanismPoses() {
    double stage1Height = Mechanisms.m_elevatorStage1Mech2d.getLength() -
        PhysicalRobotConstants.kMinElevatorStage1HeightMeters;
    double carriageHeight = Mechanisms.m_elevatorCarriageMech2d.getLength() -
        PhysicalRobotConstants.kMinElevatorCarriageHeightMeters;
    Pose3d[] poses = {
        new Pose3d(0, 0, stage1Height, Rotation3d.kZero),
        new Pose3d(0, 0, stage1Height + carriageHeight, Rotation3d.kZero),
        new Pose3d(
            -0.291779198,
            0,
            stage1Height + carriageHeight + 0.425256096,
            new Rotation3d(
                0,
                Units.degreesToRadians(Mechanisms.m_wristMech2d.getAngle() - 39.5),
                0)),
    };
    return poses;
  }

  public double getActualPosition() {
    return elevatorMotor.getEncoder().getPosition();
  }

  public Distance getLinearPosition() {
    return convertRotationsToDistance(Rotations.of(elevatorMotor.getEncoder().getPosition()));
  }

  public double getLinearPositionMeters() {
    return getLinearPosition().in(Meters);
  }

  public double getVelocityMetersPerSecond() {
    return ((elevatorEncoder.getVelocity() / 60) / PhysicalRobotConstants.kElevatorGearing) *
        (2 * Math.PI * PhysicalRobotConstants.kElevatorDrumRadius);
  }

  public double getElevatorAppliedOutput() {
    return elevatorMotor.getAppliedOutput();
  }

  public static Distance convertRotationsToDistance(Angle rotations) {
    return Meters.of(
        (rotations.in(Rotations) / PhysicalRobotConstants.kElevatorGearing) *
            (PhysicalRobotConstants.kElevatorDrumRadius * 2 * Math.PI));
  }
  public static double convertRotationsToMeters(double rotations) {
    return (rotations /
        PhysicalRobotConstants.kElevatorGearing) *
        (PhysicalRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI);
  }

  public LinearVelocity getVelocity() {
    return convertRotationsToDistance(
        Rotations.of(elevatorEncoder.getVelocity())).per(Minute);
  }

  public Command runSysIdRoutine() {
    return (sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE"));
  }

  public Distance getSetpointMeters() {
    return Meters.of(elevatorCurrentTarget.height);
  }

  public double getSetpointPose() {
    return elevatorCurrentTarget.height;
  }

  public Trigger atHeight(double height) {
    return new Trigger(() -> {
      return MathUtil.isNear(height,
          (getLinearPositionMeters()), 0.02);
    });
  }

  public Command elevatorToPosition(ElevatorState state) {
    return Commands.sequence(setSetpointCommand(state), Commands.waitUntil(atSetpoint));
  }

  public Trigger atSetpoint = new Trigger(() -> {
    return MathUtil.isNear(getLinearPositionMeters(), elevatorCurrentTarget.height, 0.02);
  });

  public Trigger atSetpoint(ElevatorState setpoint) {
    return new Trigger(() -> {
      return MathUtil.isNear(getLinearPositionMeters(), setpoint.height, 0.02);
    });
  }

}
