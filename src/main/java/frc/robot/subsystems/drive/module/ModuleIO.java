package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {}

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {}

    
}

// i need to figure out what on earth this interface should do
// does maplesim even support on smax PID?