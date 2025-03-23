// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

import choreo.trajectory.SwerveSample;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Swerve;
import frc.robot.constants.Swerve.AutoConstants;
import frc.robot.constants.Swerve.CANIds;
import frc.robot.constants.Swerve.DriveConstants;
import frc.robot.constants.Swerve.DriveSetpoints;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.vision.FiducialPoseEstimator;
import frc.robot.subsystems.vision.SwervePoseEstimator;
import frc.robot.subsystems.vision.FiducialPoseEstimator.GyroYawGetter;
import frc.robot.util.RepulsorFieldPlanner;
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

  private SlewRateLimiter accelfilterxL4 = new SlewRateLimiter(0.75);
  private SlewRateLimiter accelfilteryL4 = new SlewRateLimiter(0.75);

  public DriveSetpoints setpoint = DriveSetpoints.A;

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]"); // TODO: figure out why this is a 4 and not 0
  // like in the docs
  private SimDouble angle = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
  private ChassisSpeeds m_speedsRequested = new ChassisSpeeds();

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

  private SwervePoseEstimator poseEstimator;
  private GyroYawGetter yawGetter = (
      timestamp) -> DriverStation.isEnabled() ? yawBuffer.getSample(timestamp).orElse(null) : null;
  public final FiducialPoseEstimator[] estimators;
  // Create and configure a drivetrain simulation configuration
  private RepulsorFieldPlanner repulsorFieldPlanner = new RepulsorFieldPlanner();
  private final PIDController xController = new PIDController(
      AutoConstants.kTranslation.kP,
      AutoConstants.kTranslation.kI,
      AutoConstants.kTranslation.kD);
  private final PIDController yController = new PIDController(
      AutoConstants.kTranslation.kP,
      AutoConstants.kTranslation.kI,
      AutoConstants.kTranslation.kD);
  private final PIDController rController = new PIDController(
      AutoConstants.kRotation.kP,
      AutoConstants.kRotation.kI,
      AutoConstants.kRotation.kD);
  public Trigger atSetpoint = new Trigger(() -> xErr() <= 0.02 && yErr() <= 0.02 && aErr() <= 1).debounce(0.02);
  public Trigger atSetpointAuto = new Trigger(() -> xErr() <= 0.03 && yErr() <= 0.03 && aErr() <= 3);
  public Trigger atSetpointSource = new Trigger(() -> xErr() <= 0.04 && yErr() <= 0.04 && aErr() <= 3);
  public Trigger almostAtSetpoint = new Trigger(
      () -> {
        return getPose().minus(setpoint.getPose()).getTranslation().getNorm() < 1;
      });

  /** Creates a new DriveSubsystem. */
  public DriveTrain() {
    rController.enableContinuousInput(-Math.PI, Math.PI);
    poseEstimator = new SwervePoseEstimator(
        Swerve.DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        },
        new Pose2d());

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

        new FiducialPoseEstimator(

            "Cam_Right",
            yawGetter,
            poseEstimator::getEstimatedPosition, new Transform3d(
                Units.inchesToMeters(12.676434),
                Units.inchesToMeters(-12.438193),
                Units.inchesToMeters(9.122468),
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(20))))
    };

  }

  private double aErr() {
    return Math.abs(
        getPose().getRotation().getDegrees() - setpoint.getPose().getRotation().getDegrees())
        % 360;
  }

  private double xErr() {
    return Math.abs(getPose().getX() - setpoint.getPose().getX());
  }

  private double yErr() {
    return Math.abs(getPose().getY() - setpoint.getPose().getY());
  }

  @Override
  public void periodic() {
    Tracer.startTrace("DriveTrain");

    if (Robot.isSimulation()) {
      double dt = 0.02;

      // Update the odometry in the periodic block
      Tracer.startTrace("increment");
      m_frontLeft.incrementSim(dt);
      m_frontRight.incrementSim(dt);
      m_rearLeft.incrementSim(dt);
      m_rearRight.incrementSim(dt);
      Tracer.endTrace();
      Tracer.startTrace("angleSet");

      double dTheta = (m_speedsRequested.omegaRadiansPerSecond * dt) * 180 / Math.PI;
      angle.set(angle.get() - dTheta);
      Tracer.endTrace();

    }
    // poseEstimator.setDriveMeasurementStdDevs(getDriveStdDevs());
    if (!Robot.isSimulation()) {
      yawBuffer.addSample(RobotController.getFPGATime() / 1e6, m_gyro.getRotation2d());
      poseEstimator.setDriveMeasurementStdDevs(new double[] { 0.1, 0.1, 0.1 });
      for (var estimator : estimators) {
        var poseEstimates = estimator.poll();
        // System.out.println("poseEstimates " + poseEstimates.length);
        for (var poseEstimate : poseEstimates) {
          poseEstimator.addVisionMeasurement(
              poseEstimate.pose(),
              poseEstimate.timestamp(),
              new double[] {
                  poseEstimate.translationalStdDevs(),
                  poseEstimate.translationalStdDevs(),
                  poseEstimate.yawStdDevs()
              });

        }
      }
    }

    Tracer.startTrace("buh");
    poseEstimator.updateWithTime(
        RobotController.getTime() / 1e6,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        });
    Tracer.endTrace();
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
    setModuleStates(swerveModuleStates);
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
    m_speedsRequested = DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates);

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

  /**
   * Estimate drive wheel slippage by comparing the actual wheel velocities to the
   * idealized wheel
   * velocities. If there is a significant deviation, then a wheel(s) is slipping,
   * and we should
   * raise the estimated standard deviation of the drivebase odometry to trust the
   * wheel encoders
   * less.
   *
   * @return An array of length 3, containing the estimated standard deviations in
   *         each axis (x, y,
   *         yaw)
   */
  private double[] getDriveStdDevs() {
    var robotRelativeSpeeds = getChassisSpeeds();
    var fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, m_gyro.getRotation2d());

    var moduleStates = getSwerveModuleStates();
    // Get idealized states from the current robot velocity.
    var idealStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(getChassisSpeeds());

    double xSquaredSum = 0;
    double ySquaredSum = 0;

    for (int i = 0; i < 4; i++) {
      var measuredVector = new Translation2d(
          moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
      var idealVector = new Translation2d(idealStates[i].speedMetersPerSecond, idealStates[i].angle);

      // Compare the state vectors and get the delta between them.
      var xDelta = idealVector.getX() - measuredVector.getX();
      var yDelta = idealVector.getY() - measuredVector.getY();

      // Square the delta and add it to a sum
      xSquaredSum += xDelta * xDelta;
      ySquaredSum += yDelta * yDelta;
    }

    // Sqrt of avg of squared deltas = standard deviation
    // Rotate to convert to field relative
    double scalar = 15;
    var stdDevs = new Translation2d(
        scalar * (Math.sqrt(xSquaredSum) / 4), scalar * (Math.sqrt(ySquaredSum) / 4))
        .rotateBy(m_gyro.getRotation2d());

    // If translating and rotating at the same time, odometry drifts pretty badly in
    // the
    // direction perpendicular to the direction of translational travel.
    // This factor massively distrusts odometry in that direction when translating
    // and rotating
    // at the same time.
    var scaledSpeed = new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond)
        .rotateBy(Rotation2d.kCCW_90deg)
        .times(
            1 * Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed));

    // Add a minimum to account for mechanical slop and to prevent divide by 0
    // errors
    return new double[] {
        Math.abs(stdDevs.getX()) + Math.abs(scaledSpeed.getX()) + .1,
        Math.abs(stdDevs.getY()) + Math.abs(scaledSpeed.getY()) + .1,
        .001
    };
  }

  public Command joystickDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier,
      Optional<DoubleSupplier> heightSupplier) {
    return run(
        () -> {
          // double multiplier = (((joystick.getThrottle() * -1) + 1) / 2); // turbo mode
          double z = zSupplier.getAsDouble();
          double x = xSupplier.getAsDouble();
          double y = ySupplier.getAsDouble();

          /*
           * // limiting x/y on input methods
           * x = Math.sin(Math.atan2(x, y)) *
           * Math.min(Math.max(Math.abs(y), Math.abs(x)), 1);
           * y = Math.cos(Math.atan2(x, y)) *
           * Math.min(Math.max(Math.abs(y), Math.abs(x)), 1);
           */
          double deadband = OperatorConstants.kLogitech
              ? OperatorConstants.kLogitechDeadband
              : OperatorConstants.kDriveDeadband;

          if (heightSupplier.isPresent())
            if (MathUtil.isNear(heightSupplier.get().getAsDouble(), ElevatorState.Level4.height, 0.2)) {
              x = accelfilterxL4.calculate(x);
              y = accelfilteryL4.calculate(y);
            }

          this.drive(
              MathUtil.applyDeadband(y, deadband),
              MathUtil.applyDeadband(x, deadband),
              MathUtil.applyDeadband(z * -1, deadband),
              true);
        });
  }

  public Command autoAlignChooseSetpoint(boolean left, Optional<DoubleSupplier> xSupplier,
      Optional<DoubleSupplier> ySupplier,
      Optional<DoubleSupplier> omegaSupplier) {
    return defer(
        () -> {
          int bestBranch = 0;
          double bestScore = Double.POSITIVE_INFINITY;
          for (int i = 0; i < 6; i++) {
            var branchLocation = getBranch(i, left).getPose().getTranslation();

            var robotToBranchVector = branchLocation.minus(poseEstimator.getEstimatedPosition().getTranslation());

            var branchDistanceScore = robotToBranchVector.getNorm();
            var xControl = xSupplier.get().getAsDouble();
            var yControl = ySupplier.get().getAsDouble();
            var driverControlVector = new Translation2d(xControl, yControl);

            if (Robot.isOnRed()) {
              driverControlVector = new Translation2d(-driverControlVector.getX(), -driverControlVector.getY());
            }

            double driverInputScore;
            if (driverControlVector.getNorm() < .1) {
              driverInputScore = 0;
            } else {
              driverInputScore = dot(normalize(branchLocation), normalize(driverControlVector)) * 2.8;
            }

            DogLog.log(
                "driveTrain/Reef Align/Branch " + i + "/Distance score", branchDistanceScore);
            DogLog.log(
                "driveTrain/Reef Align/Branch " + i + "/Driver input score", driverInputScore);
            double branchScore = branchDistanceScore - driverInputScore;
            DogLog.log(
                "driveTrain/Reef Align/Branch " + i + "/Overall score", branchScore);

            if (branchScore < bestScore) {
              bestBranch = i;
              bestScore = branchScore;
            }
          }
          final var branch = getBranch(bestBranch, left);
          return autoAlign(() -> branch, xSupplier, ySupplier, omegaSupplier);
        })
        .withName("Reef align " + (left ? "left" : "right"));
  }

  private DriveSetpoints getBranch(int reefWall, boolean left) {

    return Swerve.REEF[reefWall * 2 + (left ? 0 : 1)];
  }

  public Command autoAlign(
      Supplier<DriveSetpoints> _setpoint,
      Optional<DoubleSupplier> xSupplier,
      Optional<DoubleSupplier> ySupplier,
      Optional<DoubleSupplier> omegaSupplier) {
    return run(() -> {
      if (xSupplier.isPresent() && ySupplier.isPresent() && omegaSupplier.isPresent()) {
        if (Math.abs(xSupplier.get().getAsDouble()) > 0.05
            || Math.abs(ySupplier.get().getAsDouble()) > 0.05
            || Math.abs(omegaSupplier.get().getAsDouble()) > 0.05) {
          joystickDrive(xSupplier.get(), ySupplier.get(), omegaSupplier.get(), Optional.empty()).execute();
          return;
        }
      }

      this.setpoint = _setpoint.get();
      DogLog.log("driveTrain/Setpoint", this.setpoint.getPose());

      repulsorFieldPlanner.setGoal(this.setpoint.getPose().getTranslation());

      var robotPose = getPose();
      SwerveSample cmd = repulsorFieldPlanner.getSample(
          robotPose,
          DriveConstants.kMaxSpeedMetersPerSecond);

      // Apply the trajectory with rotation adjustment
      SwerveSample adjustedSample = new SwerveSample(
          cmd.t,
          cmd.x,
          cmd.y,
          this.setpoint.getPose().getRotation().getRadians(),
          cmd.vx,
          cmd.vy,
          0,
          cmd.ax,
          cmd.ay,
          cmd.alpha,
          cmd.moduleForcesX(),
          cmd.moduleForcesY());

      // Apply the adjusted sample
      followTrajectory(adjustedSample, true);
    })
        .withName("AutoAlign");
  }

  public void followTrajectory(SwerveSample referenceState, boolean usePIDTranslation) {

    Pose2d pose = this.getPose();
    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;
    double xFeedback = 0;
    double yFeedback = 0;

    if (usePIDTranslation) {
      xFeedback = xController.calculate(pose.getX(), referenceState.x);
      yFeedback = yController.calculate(pose.getY(), referenceState.y);
    }
    double rotationFeedback = rController.calculate(
        pose.getRotation().getRadians(),
        referenceState.heading);

    ChassisSpeeds out = ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback,
        yFF + yFeedback,
        rotationFF + rotationFeedback,
        pose.getRotation());
    setChassisSpeeds(out);

  }

  public static double dot(final Translation2d a, final Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  public static Translation2d normalize(final Translation2d translation) {
    var magnitude = translation.getNorm();
    return new Translation2d(translation.getX() / magnitude, translation.getY() / magnitude);
  }

}
