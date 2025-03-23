package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import frc.robot.constants.Constants;

public class FiducialPoseEstimator {
    public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);

    private final Transform3d robotToCameraTransform;
    private final Transform2d cameraToRobotTransform2d;

    private final GyroYawGetter gyroYawGetter;
    private final Supplier<Pose2d> currentPoseEstimateSupplier;

    private final String name;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    FiducialPoseEstimate[] poseEstimates = new FiducialPoseEstimate[0];

    @FunctionalInterface
    public interface GyroYawGetter {
        Rotation2d get(double timestamp);
    }

    public FiducialPoseEstimator(
            String name,
            GyroYawGetter gyroYawGetter,
            Supplier<Pose2d> currentPoseEstimateSupplier,
            Transform3d transform) {
        this.name = name;
        this.robotToCameraTransform = transform;
        var cameraToRobotTransform3d = transform.inverse();
        cameraToRobotTransform2d = new Transform2d(
                cameraToRobotTransform3d.getTranslation().toTranslation2d(),
                cameraToRobotTransform3d.getRotation().toRotation2d());
        this.gyroYawGetter = gyroYawGetter;
        this.currentPoseEstimateSupplier = currentPoseEstimateSupplier;
        camera = new PhotonCamera(this.name);
        poseEstimator = new PhotonPoseEstimator(
                FiducialPoseEstimator.tagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraTransform);

    }

    private final ArrayList<PoseEstimate> estimatesList = new ArrayList<>();
    private final ArrayList<FiducialPoseEstimate> fiducialEstimatesList = new ArrayList<>();

    private final ArrayList<Pose3d> posesUsed = new ArrayList<>();
    private final ArrayList<Pose3d> tagsUsed = new ArrayList<>();

    public PoseEstimate[] poll() {
        var results = camera.getAllUnreadResults();

        for (var result : results) {
            poseEstimator
                    .update(result)
                    .ifPresent(
                            estimate -> {
                                var tagsUsed = new FiducialPoseEstimate.AprilTag[estimate.targetsUsed.size()];
                                for (int i = 0; i < tagsUsed.length; i++) {
                                    var tag = estimate.targetsUsed.get(i);
                                    tagsUsed[i] = new FiducialPoseEstimate.AprilTag(
                                            estimate.estimatedPose
                                                    .plus(robotToCameraTransform)
                                                    .plus(tag.bestCameraToTarget),
                                            tag.fiducialId,
                                            tag.bestCameraToTarget.getTranslation().getNorm(),
                                            tag.yaw,
                                            tag.pitch,
                                            tag.poseAmbiguity);
                                }
                                fiducialEstimatesList.add(
                                        new FiducialPoseEstimate(
                                                estimate.estimatedPose, estimate.timestampSeconds, tagsUsed));
                            });
        }

        poseEstimates = fiducialEstimatesList.toArray(new FiducialPoseEstimate[0]);
        estimatesList.clear();
        fiducialEstimatesList.clear();
        posesUsed.clear();
        tagsUsed.clear();
        for (int i = 0; i < poseEstimates.length; i++) {
            var estimate = poseEstimates[i];

            switch (estimate.tagsUsed().length) {
                case 0 -> {
                    //System.out.println("AAGH");
                } // Do nothing
                case 1 -> trigEstimate(estimate);
                default -> solvePnPEstimate(estimate);
            }
        }

        return estimatesList.toArray(new PoseEstimate[0]);
    }

    public record FiducialPoseEstimate(
            Pose3d robotPoseEstimate, double timestamp, AprilTag[] tagsUsed) {

        public record AprilTag(
                Pose3d location,
                int id,
                double distanceToCamera,
                double tx,
                double ty,
                double ambiguity) {
            static final AprilTag invalid = new AprilTag(Pose3d.kZero, -1, -1, -1, -1, -1);

            public static final Struct<AprilTag> struct = new Struct<>() {
                @Override
                public Class<AprilTag> getTypeClass() {
                    return AprilTag.class;
                }

                @Override
                public String getTypeName() {
                    return "AprilTag";
                }

                @Override
                public int getSize() {
                    return Pose3d.struct.getSize() + kSizeInt32 + kSizeDouble * 4;
                }

                @Override
                public String getSchema() {
                    return "Pose3d location;int32 id;double distanceToCamera;double tx;"
                            + "double ty;double ambiguity";
                }

                @Override
                public AprilTag unpack(ByteBuffer bb) {
                    var location = Pose3d.struct.unpack(bb);
                    var id = bb.getInt();
                    var distanceToCamera = bb.getDouble();
                    var tx = bb.getDouble();
                    var ty = bb.getDouble();
                    var ambiguity = bb.getDouble();
                    return new AprilTag(location, id, distanceToCamera, tx, ty, ambiguity);
                }

                @Override
                public void pack(ByteBuffer bb, AprilTag value) {
                    Pose3d.struct.pack(bb, value.location);
                    bb.putInt(value.id);
                    bb.putDouble(value.distanceToCamera);
                    bb.putDouble(value.tx);
                    bb.putDouble(value.ty);
                    bb.putDouble(value.ambiguity);
                }

                @Override
                public Struct<?>[] getNested() {
                    return new Struct<?>[] { Pose3d.struct };
                }
            };
        }
    }

    private void trigEstimate(FiducialPoseEstimate estimate) {

        var tag = estimate.tagsUsed()[0];

        // Fall back to SolvePnP if key data is missing (position of tag and gyro yaw)
        var tagPoseOptional = tagLayout.getTagPose(tag.id());
        var gyroYaw = gyroYawGetter.get(estimate.timestamp());
        if (tagPoseOptional.isEmpty() || gyroYaw == null) {
            //System.out.println("isEmpty");
            solvePnPEstimate(estimate);
            return;
        }
        var tagPose = tagPoseOptional.get();

        // Algorithm adapted from 6328 Mechanical Advantage
        // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
        var cameraToTagTranslation = new Pose3d(
                Translation3d.kZero,
                new Rotation3d(
                        0, Units.degreesToRadians(-tag.ty()), Units.degreesToRadians(-tag.tx())))
                .transformBy(
                        new Transform3d(new Translation3d(tag.distanceToCamera(), 0, 0), Rotation3d.kZero))
                .getTranslation()
                .rotateBy(new Rotation3d(0, robotToCameraTransform.getRotation().getY(), 0))
                .toTranslation2d();
        var cameraToTagRotation = gyroYaw.plus(
                robotToCameraTransform
                        .getRotation()
                        .toRotation2d()
                        .plus(cameraToTagTranslation.getAngle()));

        var cameraTranslation = new Pose2d(
                tagPose.getTranslation().toTranslation2d(),
                cameraToTagRotation.plus(Rotation2d.kPi))
                .transformBy(new Transform2d(cameraToTagTranslation.getNorm(), 0, Rotation2d.kZero))
                .getTranslation();
        var robotPose = new Pose2d(
                cameraTranslation,
                gyroYaw.plus(robotToCameraTransform.getRotation().toRotation2d()))
                .transformBy(cameraToRobotTransform2d);
        robotPose = new Pose2d(robotPose.getTranslation(), gyroYaw);

        // Obviously bad data falls back to SolvePnP
        if (robotPose.getX() < 0 || robotPose.getX() > Constants.FIELD_LENGTH_METERS) {
            //System.out.println("TRIG X");
            solvePnPEstimate(estimate);
            return;
        }
        if (robotPose.getY() < 0 || robotPose.getY() > Constants.FIELD_WIDTH_METERS) {
            //System.out.println("TRIG y");
            solvePnPEstimate(estimate);
            return;
        }

        // If it's too far off the pose estimate that's already in, fall back to
        // SolvePnP
        // Prevents bad data from bad initial conditions from affecting estimates
        var delta = robotPose.minus(currentPoseEstimateSupplier.get());
        if (delta.getTranslation().getNorm() > .025 || delta.getRotation().getDegrees() > 2) {
            //System.out.println("TOO FAR");

            solvePnPEstimate(estimate);
            return;
        }

        tagsUsed.add(tagPose);
        posesUsed.add(new Pose3d(robotPose));

        // Heavy distrust compared multi-tag SolvePnp, due to the inherent lack of
        // information
        // usable in the solve
        var stdDev = .5 * tag.distanceToCamera() * tag.distanceToCamera() * tag.distanceToCamera();

        estimatesList.add(new PoseEstimate(robotPose, estimate.timestamp(), stdDev, .001));
    }

    private void solvePnPEstimate(FiducialPoseEstimate estimate) {

        //System.out.println("Running solvepnp");
        // Filter out obviously bad data
        if (Math.abs(estimate.robotPoseEstimate().getZ()) > .025) {
            //System.out.println("Z" + estimate.robotPoseEstimate().getZ());
            return;

        }
        if (estimate.robotPoseEstimate().getX() < 0
                || estimate.robotPoseEstimate().getX() > Constants.FIELD_LENGTH_METERS) {
                    //System.out.println("X");

            return;
        }
        if (estimate.robotPoseEstimate().getY() < 0
                || estimate.robotPoseEstimate().getY() > Constants.FIELD_WIDTH_METERS) {
                    //System.out.println("Y");

            return;
        }
        double maxAmbiguity = .4;
        if (estimate.tagsUsed().length == 1 && estimate.tagsUsed()[0].ambiguity() > maxAmbiguity) {
            //System.out.println("Ambiguity");

            return;
        }

        double translationalScoresSum = 0;
        double angularScoresSum = 0;
        for (var tag : estimate.tagsUsed()) {
            tagsUsed.add(tag.location());
            var tagDistance = tag.distanceToCamera();

            translationalScoresSum += .4 * tagDistance * tagDistance;
            angularScoresSum += .2 * tagDistance * tagDistance;
        }

        // Heavily distrust single tag observations
        if (estimate.tagsUsed().length == 1) {
            var scale = estimate.tagsUsed()[0].ambiguity() / maxAmbiguity;
            translationalScoresSum *= MathUtil.interpolate(10, 50, scale);
            angularScoresSum *= MathUtil.interpolate(25, 100, scale);
        }

        var translationalDivisor = Math.pow(estimate.tagsUsed().length, 1.5);
        var angularDivisor = Math.pow(estimate.tagsUsed().length, 3);

        posesUsed.add(estimate.robotPoseEstimate());
        //System.out.println("made it this far");
        estimatesList.add(
                new PoseEstimate(
                        estimate.robotPoseEstimate().toPose2d(),
                        estimate.timestamp(),
                        translationalScoresSum / translationalDivisor,
                        angularScoresSum / angularDivisor));
    }

    public record PoseEstimate(
            Pose2d pose, double timestamp, double translationalStdDevs, double yawStdDevs) {
    }
}