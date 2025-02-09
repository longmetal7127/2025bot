package frc.robot;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Swerve.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import java.util.function.Consumer;

public class AutoController implements Consumer<SwerveSample> {

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
  public void accept(SwerveSample referenceState) {
    Pose2d pose = swerve.getPose();
    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;

    double xFeedback = xController.calculate(pose.getX(), referenceState.x);
    double yFeedback = yController.calculate(pose.getY(), referenceState.y);
    double rotationFeedback = rController.calculate(
      pose.getRotation().getRadians(),
      referenceState.heading
    );

    ChassisSpeeds out = ChassisSpeeds.fromFieldRelativeSpeeds(
      xFF + xFeedback,
      yFF + yFeedback,
      rotationFF + rotationFeedback,
      pose.getRotation()
    );
  }

}
