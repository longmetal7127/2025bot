package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }
    
    @Override // specified by GroIOSim interface
    public Rotation2d getGyroRotation() {
        return this.gyroSimulation.getGyroReading();
    }
    
    @Override // specified by GroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return this.gyroSimulation.getMeasuredAngularVelocity();
    }
    
}
