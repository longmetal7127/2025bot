package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.configs.Swerve.ModuleConfigs;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;

public class ModuleIOSpark implements ModuleIO {
    private final SparkFlex m_drivingSpark;
    private final SparkMax m_turningSpark;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    // sim encoders
    public final SparkRelativeEncoderSim m_drivingEncoderSim;
    public final SparkAbsoluteEncoderSim m_turningEncoderSim;

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public ModuleIOSpark(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(ModuleConfigs.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(ModuleConfigs.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;

    m_drivingEncoderSim = new SparkRelativeEncoderSim(m_drivingSpark);
    m_turningEncoderSim = new SparkAbsoluteEncoderSim(m_turningSpark);

    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
  }

    @Override
    public void setDriveOutputVoltage(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveOutputVoltage'");
    }

    @Override
    public void setSteerOutputVoltage(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSteerOutputVoltage'");
    }

    @Override
    public void requestDriveVelocity(LinearVelocity driveVelocitySetpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'requestDriveVelocity'");
    }

    @Override
    public void requestSteerPosition(Rotation2d steerFacingSetpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'requestSteerPosition'");
    }

    @Override
    public Rotation2d getSteerFacing() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSteerFacing'");
    }

    @Override
    public Angle getSteerRelativePosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSteerRelativePosition'");
    }

    @Override
    public Angle getDriveWheelrPositiond() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveWheelrPositiond'");
    }

}
