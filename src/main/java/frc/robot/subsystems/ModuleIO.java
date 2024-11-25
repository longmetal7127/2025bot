package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ModuleIO {
    public void setDriveOutputVoltage(Voltage voltage);
    public void setSteerOutputVoltage(Voltage voltage);
    public void requestDriveVelocity(LinearVelocity driveVelocitySetpoint);
    public void requestSteerPosition(Rotation2d steerFacingSetpoint);
    public Rotation2d getSteerFacing();
    public Angle getSteerRelativePosition();
    public Angle getDriveWheelrPositiond();
    
} 
