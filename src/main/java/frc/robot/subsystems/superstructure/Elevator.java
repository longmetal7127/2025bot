// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
    Processor(0.25),
    Level3(0.68),
    Level4(1.33),
    Algae1(0.653),
    Algae2(1.045),
    SourcePickup(0.2),
    Handoff(0);

    public double height;

    ElevatorState(double height) {
      this.height = height;
    }

  }

  private TalonFXS elevatorMotor = new TalonFXS(
      CANIds.kElevatorMotorCanId);
  private TalonFXS elevatorFollower = new TalonFXS(
      CANIds.kElevatorMotorFollowerCanId);

  private ElevatorState elevatorCurrentTarget = ElevatorState.Min;
  private Notifier simNotifier = null;

  private DCMotor elevatorMotorModel = DCMotor.getNEO(2);

  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      elevatorMotorModel,
      PhysicalRobotConstants.kElevatorGearing,
      PhysicalRobotConstants.kCarriageMass,
      PhysicalRobotConstants.kElevatorDrumRadius,
      0,
      PhysicalRobotConstants.kMaxElevatorStage1HeightMeters,
      false,
      0,
      0.0,
      0.0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(0.75), Volts.of(4.5), Seconds.of(10),
          (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          (volts) -> elevatorMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),

          null, this));

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
    var elevatorMotorConfigurator = elevatorMotor.getConfigurator();
    var followerConfigurator = elevatorFollower.getConfigurator();

    elevatorMotorConfigurator.apply(Configs.elevatorConfig);
    followerConfigurator.apply(Configs.elevatorFollowerConfig);
    // elevatorMotor.setControl(m_request);
    elevatorMotor.getSimState().MotorOrientation = ChassisReference.Clockwise_Positive;

    SmartDashboard.putData("elevator.mech2d", Mechanisms.m_mech2d);
    elevatorFollower.setControl(new Follower(elevatorMotor.getDeviceID(), true));

    elevatorMotor.setPosition(0);
    // Initialize simulation values
    Mechanisms.m_elevatorCarriageMech2d.setColor(new Color8Bit("#ff0000"));
    Mechanisms.m_elevatorStage1Mech2d.setColor(new Color8Bit("#00ff00"));
    if (Robot.isSimulation()) {
      simNotifier = new Notifier(() -> {
        updateSimState();
      });
      simNotifier.startPeriodic(0.02);
    }
  }

  public void updateSimState() {
    var elevatorMotorSim = elevatorMotor.getSimState();
    m_elevatorSim.setInputVoltage(
        elevatorMotorSim.getMotorVoltageMeasure().in(Volts));

    m_elevatorSim.update(0.02);
    elevatorMotorSim.setRawRotorPosition(convertMetersToRotations(m_elevatorSim.getPositionMeters()));
    elevatorMotorSim.setRotorVelocity(
        convertMetersToRotations(m_elevatorSim.getVelocityMetersPerSecond()));

  }

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

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

          m_request.withPosition(convertMetersToRotations(elevatorCurrentTarget.height));
          elevatorMotor.setControl(m_request);
          elevatorFollower.setControl(new Follower(elevatorMotor.getDeviceID(), true));

        }));
  }

  @Override
  public void periodic() {
    Tracer.startTrace("ElevatorPeriodic");

    double height = (elevatorMotor.getPosition().getValueAsDouble() /
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
        PhysicalRobotConstants.kMinElevatorStage1HeightMeters + stage1Height);

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
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public Distance getLinearPosition() {
    return convertRotationsToDistance(elevatorMotor.getPosition().getValue());
  }

  public double getLinearPositionMeters() {
    return getLinearPosition().in(Meters);
  }

  public double getVelocityMetersPerSecond() {
    return ((elevatorMotor.getVelocity().getValueAsDouble()) / PhysicalRobotConstants.kElevatorGearing) *
        (2 * Math.PI * PhysicalRobotConstants.kElevatorDrumRadius);
  }

  public double getElevatorAppliedOutput() {
    return elevatorMotor.getClosedLoopOutput().getValue();
  }

  public static Distance convertRotationsToDistance(Angle rotations) {
    return Meters.of(
        (rotations.in(Rotations) / PhysicalRobotConstants.kElevatorGearing) *
            (PhysicalRobotConstants.kElevatorDrumRadius * 2 * Math.PI));
  }

  public static LinearVelocity convertRotationsVelocityToMeters(AngularVelocity rotations) {
    return MetersPerSecond.of(
        (rotations.in(RotationsPerSecond) / PhysicalRobotConstants.kElevatorGearing) *
            (PhysicalRobotConstants.kElevatorDrumRadius * 2 * Math.PI));
  }

  public static double convertRotationsToMeters(double rotations) {
    return (rotations /
        PhysicalRobotConstants.kElevatorGearing) *
        (PhysicalRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI);
  }

  public static double convertMetersToRotations(double meters) {
    return ((meters) /
        (PhysicalRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI)) * PhysicalRobotConstants.kElevatorGearing;
  }

  public LinearVelocity getVelocity() {
    return convertRotationsVelocityToMeters(
        elevatorMotor.getVelocity().getValue());
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
  public Trigger atZeroNeedReset = new Trigger(() -> getLinearPosition().isNear(Inches.of(0), Inches.of(3))
      && elevatorMotor.getVelocity().getValue().isNear(RPM.of(0), RotationsPerSecond.of(0.005)))
      .and(atSetpoint.negate()).debounce(0.1);

  public Trigger atSetpoint(ElevatorState setpoint) {
    return new Trigger(() -> {
      return MathUtil.isNear(getLinearPositionMeters(), setpoint.height, 0.02);
    });
  }

}
