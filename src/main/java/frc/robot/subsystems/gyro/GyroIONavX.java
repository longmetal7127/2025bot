package frc.robot.subsystems.gyro;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIONavX implements GyroIO {
    private final AHRS navx;

    public GyroIONavX() {
        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
        // Implementation details
    }

    @Override // specified by GyroIOSim interface
    public Rotation2d getGyroRotation() {
        return navx.getRotation2d();
    }

    @Override // specified by GyroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return DegreesPerSecond.of(navx.getRate());
    }

}
