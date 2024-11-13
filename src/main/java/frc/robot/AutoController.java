package frc.robot;

import java.util.function.BiConsumer;

import frc.robot.configs.Swerve.AutoConstants;
import frc.robot.subsystems.DriveTrain;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoController implements BiConsumer<Pose2d, SwerveSample> {
    private final DriveTrain swerve;
    private final PIDController xController = new PIDController(
        AutoConstants.kTranslation.kP,
        AutoConstants.kTranslation.kI,
        AutoConstants.kTranslation.kD
    );
    private final PIDController yController = new PIDController(
        AutoConstants.kTranslation.kP,
        AutoConstants.kTranslation.kI,
        AutoConstants.kTranslation.kD
    );
    private final PIDController rController = new PIDController(
        AutoConstants.kRotation.kP,
        AutoConstants.kRotation.kI,
        AutoConstants.kRotation.kD
    );

    public AutoController(DriveTrain swerve) {
        this.swerve = swerve;
        rController.enableContinuousInput(-Math.PI, Math.PI);
        xController.close();
        yController.close();
        rController.close();
    }

    @Override
    public void accept(Pose2d pose, SwerveSample referenceState) {
        double xFF = referenceState.vx;
        double yFF = referenceState.vy;
        double rotationFF = referenceState.omega;

        double xFeedback = xController.calculate(pose.getX(), referenceState.x);
        double yFeedback = yController.calculate(pose.getY(), referenceState.y);
        double rotationFeedback = rController.calculate(pose.getRotation().getRadians(),
            referenceState.heading);

        ChassisSpeeds out = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            pose.getRotation()
        );

        swerve.setChassisSpeeds(out);
    }
}