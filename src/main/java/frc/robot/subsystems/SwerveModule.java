// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Swerve.ModuleConfigs;
import frc.robot.constants.Swerve.ModuleConstants;

@Logged
public class SwerveModule {

    private final SparkFlex m_drivingSpark;
    private final SparkMax m_turningSpark;
    private final SparkFlexSim m_drivingSparkSim;
    private final SparkMaxSim m_turningSparkSim;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(
            0.0,
            new Rotation2d());

    // sim encoders
    public final SparkRelativeEncoderSim m_drivingEncoderSim;
    public final SparkAbsoluteEncoderSim m_turningEncoderSim;

    private final DCMotorSim driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1),
                    Units.lbsToKilograms(115) * Math.pow(ModuleConstants.kWheelDiameterMeters / 2, 2) / 4,
                    ModuleConstants.kDrivingMotorReduction),
            DCMotor.getNeoVortex(1));

    private final DCMotorSim turnMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), .0025, ModuleConstants.kTurningMotorReduction),
            DCMotor.getNeo550(1));

    /**
     * Constructs a SwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public SwerveModule(
            int drivingCANId,
            int turningCANId,
            double chassisAngularOffset) {
        m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
        m_drivingSparkSim = new SparkFlexSim(m_drivingSpark, driveMotorSim.getGearbox());
        m_turningSparkSim = new SparkMaxSim(m_turningSpark, turnMotorSim.getGearbox());
        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        m_drivingSpark.configure(
                ModuleConfigs.drivingConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turningSpark.configure(
                ModuleConfigs.turningConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;

        m_drivingEncoderSim = new SparkRelativeEncoderSim(m_drivingSpark);
        m_turningEncoderSim = new SparkAbsoluteEncoderSim(m_turningSpark);

        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(
                m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(
                Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(
                new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingClosedLoopController.setReference(
                correctedDesiredState.speedMetersPerSecond,
                ControlType.kVelocity);
        m_turningClosedLoopController.setReference(
                correctedDesiredState.angle.getRadians(),
                ControlType.kPosition);

        m_desiredState = desiredState;

        // setting encoder values in sim
        /*
         * if (RobotBase.isSimulation()) {
         * m_drivingEncoderSim.setVelocity(
         * correctedDesiredState.speedMetersPerSecond);
         * m_turningEncoderSim.setPosition(correctedDesiredState.angle.getRadians());
         * }
         */
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }

    public void incrementSim(double dt) {
        double drivingFactor = (ModuleConstants.kWheelDiameterMeters * Math.PI);
        turnMotorSim.setInput(m_turningSparkSim.getAppliedOutput() * m_turningSparkSim.getBusVoltage());
        turnMotorSim.update(0.02);
        driveMotorSim.setInput(m_drivingSparkSim.getAppliedOutput() * m_drivingSparkSim.getBusVoltage());
        driveMotorSim.update(0.02);

        m_turningSparkSim.iterate(turnMotorSim.getAngularVelocityRadPerSec(),
                RobotController.getBatteryVoltage(), dt);

        m_turningEncoderSim.iterate(turnMotorSim.getAngularVelocityRadPerSec() / ModuleConstants.kTurningMotorReduction,
                dt);

        m_drivingSparkSim.iterate(
                driveMotorSim.getAngularVelocityRPM() * (drivingFactor / 60.0),
                RobotController.getBatteryVoltage(), dt);

    }
}
