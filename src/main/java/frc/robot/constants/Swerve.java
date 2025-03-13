package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.FiducialPoseEstimator;

public class Swerve {

  public static final class DriveConstants {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.65;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot

    public static final double kBumperWidth = Inches.of(37.75).in(Meters);
    public static final double kDriveBaseRadius = Math.sqrt(
        Math.pow(kTrackWidth / 2, 2) + Math.pow(kWheelBase / 2, 2)); // Distance from farthest wheel to center

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
  }

  public static final class CANIds {

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;
  }

  public static final class ModuleConstants {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double coefficientFriction = 1.43;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
        kDrivingMotorReduction;

    public static final double kTurningMotorReduction = 46.42;
  }

  public static final class MotorConstants {

    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class ModuleConfigs {

    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = (ModuleConstants.kWheelDiameterMeters * Math.PI) /
          ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second

      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class AutoConstants {

    public static final class kTranslation {

      public static final double kP = 3.4;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class kRotation {

      public static final double kP = 3.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class MotorConstants {
      public static final double kFreeSpeedRpm = 6784;
    }

    public static final class ModuleConfigs {
      public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
      public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

      static {
        // Use module constants to calculate conversion factors and feed forward gain.
        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
            / ModuleConstants.kDrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.04, 0, 0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1, 1);

        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second

        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);
      }
    }
  }

  public static DriveSetpoints[] REEF = { DriveSetpoints.A, DriveSetpoints.B, DriveSetpoints.C, DriveSetpoints.D,
      DriveSetpoints.E, DriveSetpoints.F, DriveSetpoints.G, DriveSetpoints.H, DriveSetpoints.I,
      DriveSetpoints.J, DriveSetpoints.K, DriveSetpoints.L };

  public static enum DriveSetpoints implements StructSerializable {
    A(FiducialPoseEstimator.tagLayout.getTagPose(18).get(), true),
    B(FiducialPoseEstimator.tagLayout.getTagPose(18).get(), false),
    C(FiducialPoseEstimator.tagLayout.getTagPose(17).get(), true),
    D(FiducialPoseEstimator.tagLayout.getTagPose(17).get(), false),
    E(FiducialPoseEstimator.tagLayout.getTagPose(22).get(), true),
    F(FiducialPoseEstimator.tagLayout.getTagPose(22).get(), false),
    G(FiducialPoseEstimator.tagLayout.getTagPose(21).get(), true),
    H(FiducialPoseEstimator.tagLayout.getTagPose(21).get(), false),
    I(FiducialPoseEstimator.tagLayout.getTagPose(20).get(), true),
    J(FiducialPoseEstimator.tagLayout.getTagPose(20).get(), false),
    K(FiducialPoseEstimator.tagLayout.getTagPose(19).get(), true),
    L(FiducialPoseEstimator.tagLayout.getTagPose(19).get(), false),
    // PROCESSOR(Pose2d.kZero),
    LEFT_HP(new Pose2d(1.18, 7.24, Rotation2d.fromDegrees(125.989 + 180))),
    RIGHT_HP(new Pose2d(1.10, 0.89, Rotation2d.fromDegrees(125.989 + 180).unaryMinus()));

    private final Pose2d pose;

    public Pose2d getPose() {
      boolean isFlipped = DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red;
      if (isFlipped) {
        return ChoreoAllianceFlipUtil.flip(pose);
      }
      return pose;
    }

    static Pose3d mapPose(Pose3d pose) {
      double angle = pose.getRotation().getAngle();
      return new Pose3d(
          pose.getX() + Math.cos(angle) * DriveConstants.kBumperWidth / 2.0,
          pose.getY() + Math.sin(angle) * DriveConstants.kBumperWidth / 2.0,
          0.0,
          pose.getRotation());
    }

    DriveSetpoints(Pose3d tag, boolean side) {
      double sideOffset = 0.156;

      Pose3d mappedPose = mapPose(tag);
      Rotation3d rotation = mappedPose.getRotation();
      double baseAngle = rotation.getAngle();
      double cos = Math.cos(baseAngle);
      double sin = Math.sin(baseAngle);
      double xOffset = sideOffset * sin;
      double yOffset = sideOffset * cos;
      if (side) {
        this.pose = new Pose2d(
            mappedPose.getX() + xOffset,
            mappedPose.getY() - yOffset,
            rotation.toRotation2d().plus(Rotation2d.kPi));
      } else {
        this.pose = new Pose2d(
            mappedPose.getX() - xOffset,
            mappedPose.getY() + yOffset,
            rotation.toRotation2d().plus(Rotation2d.kPi));
      }
    }

    DriveSetpoints(Pose2d pose) {
      this.pose = pose;
    }

    public static final Struct<DriveSetpoints> struct = StructGenerator.genEnum(DriveSetpoints.class);
  }

}
