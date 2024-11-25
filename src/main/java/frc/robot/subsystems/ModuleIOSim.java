package frc.robot.subsystems;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.ControlRequest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;


// This is only an example simulation IO implement, please change the code according to your ModuleIO interface
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        // configures the drive motor gains
        this.moduleSimulation.getDriveMotorConfigs()
            // configures a default feed-forward gains for the drive motor, with 0.1 volts of friction compensation
            .withDefaultFeedForward(Volts.of(0.1))
            // configures a velocity-voltage close loop controller
            .withVelocityVoltageController(
                // P gain is 12 volts / 3000 RPM (change it for your code)
                // Meaning that: the correction voltage is 12 volts when error is 3000RPM
                Volts.per(RPM).ofNative(12.0/3000.0), 
                // true means the P gain is specified in un-geared motor speed
                true
            );
        
        // configures the steer motor gains
        this.moduleSimulation.getSteerMotorConfigs()
            // configures a position-current close loop controller
            .withPositionCurrentController(
                // P gain is 12 amps / 60 degrees
                // Meaning that: the correction current is 20 amps when the steer position error is 60 degrees
                Amps.per(Degrees).ofNative(10.0/60.0), 
                // D gain is zero
                Amps.per(DegreesPerSecond).ofNative(0),
                // false means the P and D gain are specifed in final mechanism position/velocity
                false
            );
    }
    
    @Override // specified by ModuleIO interface
    public void setDriveOutputVoltage(Voltage voltage) {
        this.moduleSimulation.requestDriveControl(new ControlRequest.VoltageOut(voltage));
    }

    @Override // specified by ModuleIO interface
    public void setSteerOutputVoltage(Voltage voltage) {
        this.moduleSimulation.requestSteerControl(new ControlRequest.VoltageOut(voltage));
    }

    @Override // specified by ModuleIO interface
    public void requestDriveVelocity(LinearVelocity driveVelocitySetpoint) {
        // calculates the amount of wheel speed needed to achive the linear speed
        final AngularVelocity wheelVelocitySetpoint = RadiansPerSecond.of(
            driveVelocitySetpoint.in(MetersPerSecond) / moduleSimulation.WHEEL_RADIUS.in(Meters)
        );

        // request the drive motor controller to run a closed-loop on velocity
        moduleSimulation.requestDriveControl(
            new ControlRequest.VelocityVoltage(wheelVelocitySetpoint)
        );
    }

    @Override // specified by ModuleIO interface
    public void requestSteerPosition(Rotation2d steerFacingSetpoint) {
        // request the steer motor controller to run a close
        moduleSimulation.requestSteerControl(
            new ControlRequest.PositionCurrent(steerFacingSetpoint.getMeasure())
        );
    }
    
    @Override // specified by ModuleIO interface
    public Rotation2d getSteerFacing() {
        return this.moduleSimulation.getSteerAbsoluteFacing();
    }
    
    @Override // specified by ModuleIO interface
    public Angle getSteerRelativePosition() {
        return moduleSimulation.getSteerRelativeEncoderPosition().divide(moduleSimulation.STEER_GEAR_RATIO);
    }
    
    @Override // specified by ModuleIO interface
    public Angle getDriveWheelrPositiond() {
        return moduleSimulation.getDriveWheelFinalPosition();
    }
}