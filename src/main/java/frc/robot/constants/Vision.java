package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public  class Vision {
    public static class CameraConstants {
        public Transform3d robotToCameraTransform = Transform3d.kZero;

    }

    public static class CameraOne extends CameraConstants {
        public Transform3d robotToCameraTransform = new Transform3d(
                Units.inchesToMeters(-4.053308),
                Units.inchesToMeters(-9.375),
                Units.inchesToMeters(-21.108355),
                new Rotation3d(0, Units.degreesToRadians(10), 0));

    }
}
