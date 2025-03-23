// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.*;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

/**
 * A copy of {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} to allow for on-the-fly
 * drive std dev tuning.
 */
public class SwervePoseEstimator {
  private final SwerveDriveOdometry odometry;

  private static final double BUFFER_DURATION = 1.5;
  // Maps timestamps to odometry-only pose estimates
  private final TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);
  // Maps timestamps to vision updates
  // Always contains one entry before the oldest entry in odometryPoseBuffer, unless there have
  // been no vision measurements after the last reset
  private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

  private Pose2d poseEstimate;

  /**
   * Constructs a SwervePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param initialYaw The initial gyro yaw.
   * @param initialPositions The initial positions.
   * @param initialPose The initial pose.
   */
  public SwervePoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d initialYaw,
      SwerveModulePosition[] initialPositions,
      Pose2d initialPose) {
    odometry = new SwerveDriveOdometry(kinematics, initialYaw, initialPositions, initialPose);

    poseEstimate = odometry.getPoseMeters();
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
    // Reset state estimate and error covariance
    odometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
    odometryPoseBuffer.clear();
    visionUpdates.clear();
    poseEstimate = poseMeters;
  }

  /**
   * Resets the robot's pose.
   *
   * @param pose The pose to reset to.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPose(pose);
    odometryPoseBuffer.clear();
  }

  /**
   * Resets the robot's translation.
   *
   * @param translation The pose to translation to.
   */
  public void resetTranslation(Translation2d translation) {
    odometry.resetTranslation(translation);
    odometryPoseBuffer.clear();
  }

  /**
   * Resets the robot's rotation.
   *
   * @param rotation The rotation to reset to.
   */
  public void resetRotation(Rotation2d rotation) {
    odometry.resetRotation(rotation);
    odometryPoseBuffer.clear();
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return poseEstimate;
  }

  /**
   * Return the pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds The pose's timestamp in seconds.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
   */
  public Optional<Pose2d> sampleAt(double timestampSeconds) {
    // Step 0: If there are no odometry updates to sample, skip.
    if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
      return Optional.empty();
    }

    // Step 1: Make sure timestamp matches the sample from the odometry pose buffer. (When sampling,
    // the buffer will always use a timestamp between the first and last timestamps)
    double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().firstKey();
    double newestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().lastKey();
    timestampSeconds =
        MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

    // Step 2: If there are no applicable vision updates, use the odometry-only information.
    if (visionUpdates.isEmpty() || timestampSeconds < visionUpdates.firstKey()) {
      return odometryPoseBuffer.getSample(timestampSeconds);
    }

    // Step 3: Get the latest vision update from before or at the timestamp to sample at.
    double floorTimestamp = visionUpdates.floorKey(timestampSeconds);
    var visionUpdate = visionUpdates.get(floorTimestamp);

    // Step 4: Get the pose measured by odometry at the time of the sample.
    var odometryEstimate = odometryPoseBuffer.getSample(timestampSeconds);

    // Step 5: Apply the vision compensation to the odometry pose.
    return odometryEstimate.map(visionUpdate::compensate);
  }

  /** Removes stale vision updates that won't affect sampling. */
  private void cleanUpVisionUpdates() {
    // Step 0: If there are no odometry samples, skip.
    if (odometryPoseBuffer.getInternalBuffer().isEmpty()) {
      return;
    }

    // Step 1: Find the oldest timestamp that needs a vision update.
    double oldestOdometryTimestamp = odometryPoseBuffer.getInternalBuffer().firstKey();

    // Step 2: If there are no vision updates before that timestamp, skip.
    if (visionUpdates.isEmpty() || oldestOdometryTimestamp < visionUpdates.firstKey()) {
      return;
    }

    // Step 3: Find the newest vision update timestamp before or at the oldest timestamp.
    double newestNeededVisionUpdateTimestamp = visionUpdates.floorKey(oldestOdometryTimestamp);

    // Step 4: Remove all entries strictly before the newest timestamp we need.
    visionUpdates.headMap(newestNeededVisionUpdateTimestamp, false).clear();
  }

  private double[] driveStdDevs = new double[3];
  private boolean driveMatrixInitialized = false;

  /**
   * @param driveMeasurementStdDevs Standard deviations of the pose estimate (x position in meters,
   *     y position in meters, and heading in radians). Increase these numbers to trust your state
   *     estimate less.
   */
  public final void setDriveMeasurementStdDevs(double[] driveMeasurementStdDevs) {
    if (driveMeasurementStdDevs.length != 3) {
      throw new IllegalArgumentException("Length of driveMeasurementStdDevs must be 3!");
    }
    driveStdDevs = driveMeasurementStdDevs;
    driveMatrixInitialized = true;
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * #updateWithTime} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   */
  public void addVisionMeasurement(Pose2d poseEstimate, double timestamp, double[] stdDevs) {
    if (!driveMatrixInitialized) {
      throw new NullPointerException("Drive standard deviation matrix not initialized!");
    }

    // Update vision matrix
    // Step 0: square each term
    var r = new double[3];
    for (int i = 0; i < 3; i++) {
      r[i] = stdDevs[i] * stdDevs[i];
    }

    // Step 1: solve for closed form Kalman gain for continuous Kalman filter with A = 0 and C =
    // I. See wpimath/algorithms.md.
    var visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      if (driveStdDevs[row] == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(
            row,
            row,
            driveStdDevs[row] / (driveStdDevs[row] + Math.sqrt(driveStdDevs[row] * r[row])));
      }
    }

    // Update pose estimate
    // Step 0: If this measurement is old enough to be outside the pose buffer's timespan, skip.
    if (odometryPoseBuffer.getInternalBuffer().isEmpty()
        || odometryPoseBuffer.getInternalBuffer().lastKey() - BUFFER_DURATION > timestamp) {
      return;
    }

    // Step 1: Clean up any old entries
    cleanUpVisionUpdates();

    // Step 2: Get the pose measured by odometry at the moment the vision measurement was made.
    var odometrySample = odometryPoseBuffer.getSample(timestamp);

    if (odometrySample.isEmpty()) {
      return;
    }

    // Step 3: Get the vision-compensated pose estimate at the moment the vision measurement was
    // made.
    var visionSample = sampleAt(timestamp);

    if (visionSample.isEmpty()) {
      return;
    }

    // Step 4: Measure the twist between the old pose estimate and the vision pose.
    var twist = visionSample.get().log(poseEstimate);

    // Step 5: We should not trust the twist entirely, so instead we scale this twist by a Kalman
    // gain matrix representing how much we trust vision measurements compared to our current pose.
    var k_times_twist = visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    // Step 6: Convert back to Twist2d.
    var scaledTwist =
        new Twist2d(k_times_twist.get(0, 0), k_times_twist.get(1, 0), k_times_twist.get(2, 0));

    // Step 7: Calculate and record the vision update.
    var visionUpdate = new VisionUpdate(visionSample.get().exp(scaledTwist), odometrySample.get());
    visionUpdates.put(timestamp, visionUpdate);

    // Step 8: Remove later vision measurements. (Matches previous behavior)
    visionUpdates.tailMap(timestamp, false).entrySet().clear();

    // Step 9: Update latest pose estimate. Since we cleared all updates after this vision update,
    // it's guaranteed to be the latest vision update.
    this.poseEstimate = visionUpdate.compensate(odometry.getPoseMeters());
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyro angle.
   * @param wheelPositions The current encoder readings.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {

    var odometryEstimate = odometry.update(gyroAngle, wheelPositions);

    odometryPoseBuffer.addSample(currentTimeSeconds, odometryEstimate);

    if (visionUpdates.isEmpty()) {
      poseEstimate = odometryEstimate;
    } else {
      var visionUpdate = visionUpdates.get(visionUpdates.lastKey());
      poseEstimate = visionUpdate.compensate(odometryEstimate);
    }

    return getEstimatedPosition();
  }

  /**
   * Represents a vision update record. The record contains the vision-compensated pose estimate as
   * well as the corresponding odometry pose estimate.
   *
   * @param visionPose The vision-compensated pose estimate.
   * @param odometryPose The pose estimated based solely on odometry.
   */
  private record VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
    /**
     * Returns the vision-compensated version of the pose. Specifically, changes the pose from being
     * relative to this record's odometry pose to being relative to this record's vision pose.
     *
     * @param pose The pose to compensate.
     * @return The compensated pose.
     */
    public Pose2d compensate(Pose2d pose) {
      var delta = pose.minus(this.odometryPose);
      return this.visionPose.plus(delta);
    }
  }
}