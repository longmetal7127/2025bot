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
import frc.robot.util.Tracer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.Superstructure.Setpoints;
import frc.robot.Robot;
import frc.robot.configs.Superstructure.CANIds;
import frc.robot.configs.Superstructure.Configs;
import frc.robot.configs.Superstructure.ElevatorConstants;
import frc.robot.configs.Superstructure.Mechanisms;
import frc.robot.configs.Superstructure.PhysicalRobotConstants;
import static edu.wpi.first.units.Units.*;

@Logged
public class Elevator extends SubsystemBase {

        ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(
                        ElevatorConstants.kElevatorkS,
                        ElevatorConstants.kElevatorkG,
                        ElevatorConstants.kElevatorkV,
                        ElevatorConstants.kElevatorkA);

        private SparkMax elevatorMotor = new SparkMax(CANIds.kElevatorMotorCanId, MotorType.kBrushless);
        private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
        private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

        private SparkMax elevatorFollowerMotor = new SparkMax(CANIds.kElevatorMotorFollowerCanId, MotorType.kBrushless);

        private double elevatorCurrentTarget = Setpoints.kZero;
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


        // Mechanism2d setup for subsystem

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
                m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

                m_elevatorSim.update(0.0050);

                elevatorMotorSim.iterate(
                                ((m_elevatorSim.getVelocityMetersPerSecond()
                                                / (PhysicalRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                                                * PhysicalRobotConstants.kElevatorGearing)
                                                * 60.0,
                                RobotController.getBatteryVoltage(),
                                0.005);

        }

        private void moveToSetpoint() {
                elevatorClosedLoopController.setReference(
                                elevatorCurrentTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                                m_ElevatorFeedforward.calculate(getVelocity().in(MetersPerSecond)));
        }

        public Command incrementSetpointCommand(double setpoint) {
                return this.runOnce(() -> {
                        this.elevatorCurrentTarget += setpoint;
                });
        }

        public Command setSetpointCommand(double setpoint) {
                return Commands.sequence(
                                this.runOnce(() -> {
                                        this.elevatorCurrentTarget = setpoint;

                                })

                );
        }

        @Override
        public void periodic() {
                Tracer.startTrace("ElevatorPeriodic");
                moveToSetpoint();
                double height = (elevatorEncoder.getPosition() / PhysicalRobotConstants.kElevatorGearing)
                                * (PhysicalRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI);
                double carriageHeight = Math.min(PhysicalRobotConstants.kCarriageTravelHeightMeters, height);
                double stage1Height = (carriageHeight == PhysicalRobotConstants.kCarriageTravelHeightMeters)
                                ? height - PhysicalRobotConstants.kCarriageTravelHeightMeters
                                : 0;
                Mechanisms.m_elevatorCarriageMech2d
                                .setLength(PhysicalRobotConstants.kMinElevatorCarriageHeightMeters
                                                + carriageHeight);
                                                Mechanisms.m_elevatorStage1Mech2d
                                .setLength(PhysicalRobotConstants.kMinElevatorStage1HeightMeters
                                                + stage1Height);

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
                double stage1Height = Mechanisms.m_elevatorStage1Mech2d.getLength()
                                - PhysicalRobotConstants.kMinElevatorStage1HeightMeters;
                double carriageHeight = Mechanisms.m_elevatorCarriageMech2d.getLength()
                                - PhysicalRobotConstants.kMinElevatorCarriageHeightMeters;
                Pose3d[] poses = {
                                new Pose3d(0, 0, stage1Height,
                                                Rotation3d.kZero),
                                new Pose3d(0, 0,
                                                stage1Height + carriageHeight,
                                                Rotation3d.kZero),
                                new Pose3d(-0.291779198, 0,
                                                stage1Height + carriageHeight + 0.425256096,
                                                new Rotation3d(0, Units.degreesToRadians(Mechanisms.m_armMech2d.getAngle()), 0))

                };
                return poses;
        }

        public double getActualPosition() {
                return elevatorMotor.getEncoder().getPosition();
        }

        public double getElevatorAppliedOutput() {
                return elevatorMotor.getAppliedOutput();
        }

        public static Distance convertRotationsToDistance(Angle rotations) {
                return Meters.of((rotations.in(Rotations) / PhysicalRobotConstants.kElevatorGearing) *
                                (PhysicalRobotConstants.kElevatorDrumRadius * 2 * Math.PI));
        }

        public LinearVelocity getVelocity() {
                return convertRotationsToDistance(Rotations.of(elevatorEncoder.getVelocity())).per(Minute);
        }

}