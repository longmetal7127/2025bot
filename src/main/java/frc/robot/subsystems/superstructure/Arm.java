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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Superstructure.ArmConstants;
import frc.robot.constants.Superstructure.CANIds;
import frc.robot.constants.Superstructure.Configs;
import frc.robot.constants.Superstructure.Mechanisms;
import frc.robot.constants.Superstructure.PhysicalRobotConstants;
import frc.robot.constants.Superstructure.Setpoints;
import frc.robot.util.Tracer;

@Logged
public class Arm extends SubsystemBase {

  private SparkMax armMotor = new SparkMax(
    CANIds.kArmMotorCanId,
    MotorType.kBrushless
  );
  private final ProfiledPIDController m_armPIDController =
    new ProfiledPIDController(
      ArmConstants.kArmkP,
      ArmConstants.kArmkI,
      ArmConstants.kArmkD,
      new Constraints(
        ArmConstants.kArmMaxVelocityRPM,
        ArmConstants.kArmMaxAccelerationRPMperSecond
      )
    );
  private final ArmFeedforward m_armFeedforward = new ArmFeedforward(
    ArmConstants.kArmkS,
    ArmConstants.kArmkG,
    ArmConstants.kArmkV,
    ArmConstants.kArmkV
  );

  private RelativeEncoder armEncoder = armMotor.getEncoder();

  private double armCurrentTarget = Setpoints.kZero;
  private Notifier simNotifier = null;

  private DCMotor armMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim armMotorSim;
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
    armMotorModel,
    PhysicalRobotConstants.kArmReduction,
    SingleJointedArmSim.estimateMOI(
      PhysicalRobotConstants.kArmLength.in(Meters),
      PhysicalRobotConstants.kArmMass.in(Kilograms)
    ),
    PhysicalRobotConstants.kArmLength.in(Meters),
    PhysicalRobotConstants.kMinAngleRads,
    PhysicalRobotConstants.kMaxAngleRads,
    true,
    PhysicalRobotConstants.kMinAngleRads,
    0.0,
    0.0
  );

  // Mechanism2d setup for subsystem

  public Arm() {
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
      PersistMode.kPersistParameters
    );

    armEncoder.setPosition(0);

    // Initialize simulation values
    armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
    if (Robot.isSimulation()) {
      simNotifier = new Notifier(() -> {
        updateSimState();
      });
      simNotifier.startPeriodic(0.005);
    }
  }

  public void updateSimState() {
    m_armSim.setInput(
      armMotor.getAppliedOutput() * RobotController.getBatteryVoltage()
    );

    m_armSim.update(0.0050);

    armMotorSim.iterate(
      Units.radiansPerSecondToRotationsPerMinute(
        m_armSim.getVelocityRadPerSec() * PhysicalRobotConstants.kArmReduction
      ),
      RobotController.getBatteryVoltage(),
      0.005
    );
  }

  private void moveToSetpoint() {
    double pidOutput = m_armPIDController.calculate(
      armEncoder.getPosition(),
      Units.degreesToRotations(armCurrentTarget) *
      PhysicalRobotConstants.kArmReduction
    );
    State setpointState = m_armPIDController.getSetpoint();
    armMotor.setVoltage(
      pidOutput +
      m_armFeedforward.calculate(setpointState.position, setpointState.velocity)
    );
  }

  public Command setSetpointCommand(double armSetpoint) {
    return this.runOnce(() -> {
        this.armCurrentTarget = armSetpoint;
      });
  }

  @Override
  public void periodic() {
    Tracer.startTrace("ArmPeriodic");
    moveToSetpoint();

    Mechanisms.m_armMech2d.setAngle(
      180 -
      (Units.radiansToDegrees(PhysicalRobotConstants.kMinAngleRads) +
        Units.rotationsToDegrees(
          armEncoder.getPosition() / PhysicalRobotConstants.kArmReduction
        )) -
      90
    );
    Tracer.endTrace();
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_armSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {}

  public double getArmActualPosition() {
    return armMotor.getEncoder().getPosition();
  }
}
