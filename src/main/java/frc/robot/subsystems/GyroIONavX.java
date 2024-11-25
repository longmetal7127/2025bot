package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIONavX implements GyroIO {

    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    public GyroIONavX() {

    }
    
    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return m_gyro.getRotation2d();
    }
    
    @Override // specified by GroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return DegreesPerSecond.of(m_gyro.getRate());
    }
    
}
