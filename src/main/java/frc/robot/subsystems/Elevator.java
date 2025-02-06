// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configs.Elevator.Setpoints;
import frc.robot.Robot;
import frc.robot.configs.Elevator.CANIds;
import frc.robot.configs.Elevator.Configs;
import frc.robot.configs.Elevator.ElevatorConstants;
import frc.robot.configs.Elevator.SimulationRobotConstants;
import static edu.wpi.first.units.Units.*;

@Logged
public class Elevator extends SubsystemBase {

        private SparkMax armMotor = new SparkMax(CANIds.kArmMotorCanId, MotorType.kBrushless);
        private SparkClosedLoopController armController = armMotor.getClosedLoopController();
        private RelativeEncoder armEncoder = armMotor.getEncoder();
        ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
                        ElevatorConstants.kElevatorkS,
                        ElevatorConstants.kElevatorkG,
                        ElevatorConstants.kElevatorkV,
                        ElevatorConstants.kElevatorkA);

        private SparkMax elevatorMotor = new SparkMax(CANIds.kElevatorMotorCanId, MotorType.kBrushless);
        private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
        private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

        private SparkMax elevatorFollowerMotor = new SparkMax(CANIds.kElevatorMotorFollowerCanId, MotorType.kBrushless);

        private double elevatorCurrentTarget = Setpoints.kZero;
        private double armCurrentTarget = Setpoints.kZero;
        private Notifier simNotifier = null;

        private DCMotor elevatorMotorModel = DCMotor.getNEO(2);
        private SparkMaxSim elevatorMotorSim;
        private final ElevatorSim m_elevatorSim = new ElevatorSim(
                        elevatorMotorModel,
                        SimulationRobotConstants.kElevatorGearing,
                        SimulationRobotConstants.kCarriageMass,
                        SimulationRobotConstants.kElevatorDrumRadius,
                        SimulationRobotConstants.kMinElevatorCarriageHeightMeters,
                        SimulationRobotConstants.kMaxElevatorStage1HeightMeters,
                        true,
                        0,
                        0.0,
                        0.0);

        private DCMotor armMotorModel = DCMotor.getNEO(1);
        private SparkMaxSim armMotorSim;
        private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
                        armMotorModel,
                        SimulationRobotConstants.kArmReduction,
                        SingleJointedArmSim.estimateMOI(
                                        SimulationRobotConstants.kArmLength.in(Meters),
                                        SimulationRobotConstants.kArmMass.in(Kilograms)),
                        SimulationRobotConstants.kArmLength.in(Meters),
                        SimulationRobotConstants.kMinAngleRads,
                        SimulationRobotConstants.kMaxAngleRads,
                        true,
                        SimulationRobotConstants.kMinAngleRads,
                        0.0,
                        0.0);

        // Mechanism2d setup for subsystem
        public final Mechanism2d m_mech2d = new Mechanism2d(10, 10);
        private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
        private final MechanismLigament2d m_elevatorStage1Mech2d = m_mech2dRoot.append(
                        new MechanismLigament2d(
                                        "Elevator Stage 1",
                                        SimulationRobotConstants.kMinElevatorStage1HeightMeters,
                                        90));
        private final MechanismLigament2d m_elevatorCarriageMech2d = m_elevatorStage1Mech2d.append(
                        new MechanismLigament2d(
                                        "Elevator 6Carriage",
                                        SimulationRobotConstants.kMinElevatorCarriageHeightMeters,
                                        0));

        private final MechanismLigament2d m_armMech2d = m_elevatorCarriageMech2d.append(
                        new MechanismLigament2d(
                                        "Arm",
                                        SimulationRobotConstants.kArmLength.in(Meters),
                                        180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 180));

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
                armMotor.configure(
                                Configs.armConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                elevatorMotor.configure(
                                Configs.elevatorConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                elevatorFollowerMotor.configure(
                                Configs.elevatorFollowerConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                SmartDashboard.putData("elevator.mech2d", m_mech2d);

                armEncoder.setPosition(0);
                elevatorEncoder.setPosition(0);

                // Initialize simulation values
                elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorMotorModel);
                armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
                m_elevatorCarriageMech2d.setColor(new Color8Bit("#ff0000"));
                m_elevatorStage1Mech2d.setColor(new Color8Bit("#00ff00"));
                if (Robot.isSimulation()) {
                        simNotifier = new Notifier(() -> {
                                updateSimState();
                        });
                        simNotifier.startPeriodic(0.005);
                }

        }

        public void updateSimState() {
                m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
                m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

                m_elevatorSim.update(0.0050);
                m_armSim.update(0.0050);

                elevatorMotorSim.iterate(
                                ((m_elevatorSim.getVelocityMetersPerSecond()
                                                / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                                                * SimulationRobotConstants.kElevatorGearing)
                                                * 60.0,
                                RobotController.getBatteryVoltage(),
                                0.005);
                armMotorSim.iterate(
                                Units.radiansPerSecondToRotationsPerMinute(
                                                m_armSim.getVelocityRadPerSec()
                                                                * SimulationRobotConstants.kArmReduction),
                                RobotController.getBatteryVoltage(),
                                0.005);

        }

        private void moveToSetpoint() {
                elevatorClosedLoopController.setReference(
                                elevatorCurrentTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                                m_feedforward.calculate(getVelocity().in(MetersPerSecond)));
                armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        }

        public Command incrementSetpointCommand(double setpoint) {
                return this.runOnce(() -> {
                        this.elevatorCurrentTarget += setpoint;
                });
        }

        public Command setSetpointCommand(double setpoint, double armSetpoint) {
                return Commands.sequence(
                                this.runOnce(() -> {
                                        this.armCurrentTarget = armSetpoint;
                                }),
                                Commands.waitUntil(
                                                new Trigger(() -> {
                                                        return true;
                                                        /*
                                                         * return MathUtil.isNear(armSetpoint, armEncoder.getPosition(),
                                                         * 5);
                                                         */
                                                })),
                                this.runOnce(() -> {
                                        this.elevatorCurrentTarget = setpoint;

                                })

                );
        }

        @Override
        public void periodic() {
                moveToSetpoint();
                double height = (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI);
                double carriageHeight = Math.min(SimulationRobotConstants.kCarriageTravelHeightMeters, height);
                double stage1Height = (carriageHeight == SimulationRobotConstants.kCarriageTravelHeightMeters)
                                ? height - SimulationRobotConstants.kCarriageTravelHeightMeters
                                : 0;
                m_elevatorCarriageMech2d
                                .setLength(SimulationRobotConstants.kMinElevatorCarriageHeightMeters
                                                + carriageHeight);
                m_elevatorStage1Mech2d
                                .setLength(SimulationRobotConstants.kMinElevatorStage1HeightMeters
                                                + stage1Height);

                m_armMech2d.setAngle(
                                180
                                                - (Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                                                                + Units.rotationsToDegrees(
                                                                                armEncoder.getPosition()
                                                                                                / SimulationRobotConstants.kArmReduction))
                                                - 90);

        }

        /** Get the current drawn by each simulation physics model */
        public double getSimulationCurrentDraw() {
                return m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps();
        }

        @Override
        public void simulationPeriodic() {
                // In this method, we update our simulation of what our elevator is doing
                // First, we set our "inputs" (voltages)

        }

        public Pose3d[] getMechanismPoses() {
                double stage1Height = m_elevatorStage1Mech2d.getLength()
                                - SimulationRobotConstants.kMinElevatorStage1HeightMeters;
                double carriageHeight = m_elevatorCarriageMech2d.getLength()
                                - SimulationRobotConstants.kMinElevatorCarriageHeightMeters;
                Pose3d[] poses = {
                                new Pose3d(0, 0, stage1Height,
                                                Rotation3d.kZero),
                                new Pose3d(0, 0,
                                                stage1Height + carriageHeight,
                                                Rotation3d.kZero),
                                new Pose3d(-0.291779198, 0,
                                                stage1Height + carriageHeight + 0.425256096,
                                                new Rotation3d(0, Units.degreesToRadians(m_armMech2d.getAngle()), 0))

                };
                return poses;
        }

        public double getActualPosition() {
                return elevatorMotor.getEncoder().getPosition();
        }

        public double getArmActualPosition() {
                return armMotor.getEncoder().getPosition();
        }

        public double getElevatorAppliedOutput() {
                return elevatorMotor.getAppliedOutput();
        }

        public static Distance convertRotationsToDistance(Angle rotations) {
                return Meters.of((rotations.in(Rotations) / SimulationRobotConstants.kElevatorGearing) *
                                (SimulationRobotConstants.kElevatorDrumRadius * 2 * Math.PI));
        }

        public LinearVelocity getVelocity() {
                return convertRotationsToDistance(Rotations.of(elevatorEncoder.getVelocity())).per(Minute);
        }

}