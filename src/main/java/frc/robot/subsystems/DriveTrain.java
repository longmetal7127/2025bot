// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Swerve;
import frc.robot.constants.Vision;
import frc.robot.constants.Swerve.CANIds;
import frc.robot.constants.Swerve.DriveConstants;
import frc.robot.constants.Vision.CameraOne;
import frc.robot.subsystems.vision.FiducialPoseEstimator;
import frc.robot.subsystems.vision.FiducialPoseEstimator.GyroYawGetter;
import frc.robot.subsystems.vision.FiducialPoseEstimator.PoseEstimate;
import frc.robot.util.Tracer;

@Logged
public class DriveTrain extends SubsystemBase {

  // Create SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      CANIds.kFrontLeftDrivingCanId,
      CANIds.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      CANIds.kFrontRightDrivingCanId,
      CANIds.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      CANIds.kRearLeftDrivingCanId,
      CANIds.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      CANIds.kRearRightDrivingCanId,
      CANIds.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);
  private final TimeInterpolatableBuffer<Rotation2d> yawBuffer = TimeInterpolatableBuffer.createBuffer(.25);

  // sim zero timer
  private double bufferZeroTime = 0.25;

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]"); // TODO: figure out why this is a 4 and not 0
  // like in the docs
  private SimDouble angle = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
      });

  // sim field
  private Field2d m_field = new Field2d();

  private SwerveDrivePoseEstimator poseEstimator;
  private GyroYawGetter yawGetter = (
      timestamp) -> DriverStation.isEnabled() ? yawBuffer.getSample(timestamp).orElse(null) : null;
  public final FiducialPoseEstimator[] estimators;
  // Create and configure a drivetrain simulation configuration

  /** Creates a new DriveSubsystem. */
  public DriveTrain() {
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    poseEstimator = new SwerveDrivePoseEstimator(
        Swerve.DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        },
        new Pose2d(), stateStdDevs, visionStdDevs);

    if (Robot.isSimulation()) {
      // create field on smart dashboard
      SmartDashboard.putData("Field", m_field);

      // creation the swerve simulation (please refer to previous documents)
      // resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI/2)));
      // angle.set(Math.PI/2);
      // zeroHeading();
      m_odometry.resetPose(
          new Pose2d(new Translation2d(0, 0), new Rotation2d(Math.PI / 2)));
    }
    estimators = new FiducialPoseEstimator[] { new FiducialPoseEstimator(

        "Cam_Left",
        yawGetter,
        poseEstimator::getEstimatedPosition, new Transform3d(
            Units.inchesToMeters(4.053308),
            Units.inchesToMeters(9.375),
            Units.inchesToMeters(21.108355),
            new Rotation3d(0, Units.degreesToRadians(10), 0))),
        /*new FiducialPoseEstimator(

            "Cam_Right",
            yawGetter,
            poseEstimator::getEstimatedPosition, new Transform3d(
                4.053308,
                9.375,
                21.108355,
                new Rotation3d(0, 10, 0)))*/ };

  }

  @Override
  public void periodic() {
    Tracer.startTrace("DriveTrain");
    yawBuffer.addSample(RobotController.getFPGATime() / 1e6, m_gyro.getRotation2d());
    for (var estimator : estimators) {
      var poseEstimates = estimator.poll();
      System.out.println("poseEstimates " + poseEstimates.length);
      for (var poseEstimate : poseEstimates) {
        poseEstimator.addVisionMeasurement(
            poseEstimate.pose(),
            poseEstimate.timestamp()
        /*
         * new double[] {
         * poseEstimate.translationalStdDevs(),
         * poseEstimate.translationalStdDevs(),
         * poseEstimate.yawStdDevs()
         * });
         */);
      }
    }

    // sim case
    if (Robot.isSimulation()) {
      double dt = 0.02;

      // Update the odometry in the periodic block
      m_frontLeft.incrementSim(dt);
      m_frontRight.incrementSim(dt);
      m_rearLeft.incrementSim(dt);
      m_rearRight.incrementSim(dt);

      // get velocities of x, y, and rot
      /*
       * ChassisSpeeds speeds =
       * Swerve.DriveConstants.kDriveKinematics.toChassisSpeeds(
       * new SwerveModuleState[] {
       * m_frontLeft.getState(),
       * m_frontRight.getState(),
       * m_rearLeft.getState(),
       * m_rearRight.getState(),
       * });
       * m_odometry.resetPose(
       * m_odometry
       * .getPoseMeters()
       * .exp(
       * new Twist2d(
       * speeds.vxMetersPerSecond * dt,
       * speeds.vyMetersPerSecond * dt,
       * speeds.omegaRadiansPerSecond * dt)));
       */
      // update robot pose
      angle.set(-m_odometry.getPoseMeters().getRotation().getDegrees());

    }

    poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        });

    Tracer.endTrace();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState(),
    };
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        },
        pose);
  }

  public Command cmdResetOdometry(Pose2d pose) {
    return this.runOnce(() -> {
      resetOdometry(pose);
    });
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed,
      double ySpeed,
      double rot,
      boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates,
        DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return runOnce(() -> {
      m_gyro.reset();
    });
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the current ChassisSpeeds
   *
   * @return the current chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Sets each module to the respective chassis speed
   *
   * @param speeds Desired ChassisSpeeds object
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }
}
/*
 * public Pose2d[] getSwerveModulePoses(Pose2d robotPose) {
 * Pose2d[] poseArr = new Pose2d[swerveDriveConfiguration.moduleCount];
 * List<Pose2d> poses = new ArrayList<>();
 * for (SwerveModule module : swerveModules) {
 * poses.add(
 * robotPose.plus(
 * new Transform2d(module.configuration.moduleLocation,
 * module.getState().angle)));
 * }
 * return poses.toArray(poseArr);
 * }
 */
