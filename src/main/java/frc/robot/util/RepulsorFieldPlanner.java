package frc.robot.util;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;
import java.util.ArrayList;
import java.util.List;
import dev.doglog.DogLog;

public class RepulsorFieldPlanner {
  abstract static class Obstacle {
    double strength = 1.0;
    boolean positive = true;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Force getForceAtPosition(Translation2d position, Translation2d target);

    protected double distToForceMag(double dist) {
      var forceMag = strength / (0.00001 + Math.abs(dist * dist));
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }

    protected double distToForceMag(double dist, double falloff) {
      var original = strength / (0.00001 + Math.abs(dist * dist));
      var falloffMag = strength / (0.00001 + Math.abs(falloff * falloff));
      return Math.max(original - falloffMag, 0) * (positive ? 1 : -1);
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > 4) {
        return new Force();
      }
      var outwardsMag = distToForceMag(loc.getDistance(position) - radius);
      var initial = new Force(outwardsMag, position.minus(loc).getAngle());
      var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      return initial
          .rotateBy(Rotation2d.kCCW_90deg)
          .div(initial.getNorm())
          .times(mag)
          .plus(initial);
    }
  }

  static class SnowmanObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;

    public SnowmanObstacle(Translation2d loc, double strength, double radius, boolean positive) {
      super(strength, positive);
      this.loc = loc;
      this.radius = radius;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      // 1 meter away from loc, opposite target.
      var sidewaysCircle = new Translation2d(1, targetToLoc.getAngle()).plus(loc);
      var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position));
      var outwardsMag = distToForceMag(Math.max(0.01, loc.getDistance(position) - radius));
      var initial = new Force(outwardsMag, position.minus(loc).getAngle());

      // flip the sidewaysMag based on which side of the goal-sideways circle the
      // robot is on
      var sidewaysTheta = target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());

      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
      return new Force(sideways, sidewaysAngle).plus(initial);
    }
  }

  static class HorizontalObstacle extends Obstacle {
    double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(0, distToForceMag(y - position.getY(), 1));
    }
  }

  static class VerticalObstacle extends Obstacle {
    double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      return new Force(distToForceMag(x - position.getX(), 1), 0);
    }
  }

  private static class TeardropObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double primaryRadius;
    final double tailStrength;
    final double tailLength;

    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailLength = tailLength + primaryMaxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysPoint = new Translation2d(tailLength, targetToLoc.getAngle()).plus(loc);

      var positionToLocation = position.minus(loc);
      var positionToLocationDistance = positionToLocation.getNorm();
      Translation2d outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        outwardsForce = new Translation2d(
            distToForceMag(
                Math.max(positionToLocationDistance - primaryRadius, 0),
                primaryMaxRange - primaryRadius),
            positionToLocation.getAngle());
      } else {
        outwardsForce = Translation2d.kZero;
      }

      var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
      var distanceAlongLine = positionToLine.getX();

      Translation2d sidewaysForce;
      var distanceScalar = distanceAlongLine / tailLength;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        var secondaryMaxRange = MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        var distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          double strength;
          if (distanceAlongLine < primaryMaxRange) {
            strength = tailStrength * (distanceAlongLine / primaryMaxRange);
          } else {
            strength = -tailStrength * distanceAlongLine / (tailLength - primaryMaxRange)
                + tailLength * tailStrength / (tailLength - primaryMaxRange);
          }
          strength *= 1 - distanceToLine / secondaryMaxRange;

          var sidewaysMag = tailStrength * strength * (secondaryMaxRange - distanceToLine);
          // flip the sidewaysMag based on which side of the goal-sideways circle the
          // robot is on
          var sidewaysTheta = target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
          sidewaysForce = new Translation2d(
              sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
              targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Translation2d.kZero;
        }
      } else {
        sidewaysForce = Translation2d.kZero;
      }

      return new Force(
          outwardsForce.plus(sidewaysForce).getNorm(),
          outwardsForce.plus(sidewaysForce).getAngle());
    }
  }

  public static final double GOAL_STRENGTH = 1.2;

  public static final List<Obstacle> FIELD_OBSTACLES = List.of(
      new TeardropObstacle(Constants.BLUE_REEF, 1, 2.5, .83, 3, 2),
      new TeardropObstacle(Constants.RED_REEF, 1, 2.5, .83, 3, 2));

  public static final List<Obstacle> WALLS = List.of(
      new HorizontalObstacle(0.0, 2, true),
      new HorizontalObstacle(Constants.FIELD_WIDTH_METERS, 1.4, false),
      new VerticalObstacle(0.0, 2, true),
      new VerticalObstacle(Constants.FIELD_LENGTH_METERS, 1.4, false));

  private List<Obstacle> fixedObstacles = new ArrayList<>();
  private Translation2d goal = Translation2d.kZero;

  private static final int ARROWS_X = RobotBase.isSimulation() ? 40 : 0;
  private static final int ARROWS_Y = RobotBase.isSimulation() ? 20 : 0;
  private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);

  private ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

  public RepulsorFieldPlanner() {
    fixedObstacles.addAll(FIELD_OBSTACLES);
    fixedObstacles.addAll(WALLS);
    for (int i = 0; i < ARROWS_SIZE; i++) {
      arrows.add(new Pose2d());
    }
    NetworkTableInstance.getDefault()
        .startEntryDataLog(
            DataLogManager.getLog(), "SmartDashboard/Alerts", "SmartDashboard/Alerts");
  }

  private Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  // A grid of arrows drawn in AScope
  void updateArrows() {
    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation = new Translation2d(x * Constants.FIELD_LENGTH_METERS / ARROWS_X,
            y * Constants.FIELD_WIDTH_METERS / ARROWS_Y);
        var force = getForce(translation, goal);

        if (force.getNorm() < 1e-6) {
          arrows.set(x * (ARROWS_Y + 1) + y, arrowBackstage);
        } else {
          var rotation = force.getAngle();

          arrows.set(x * (ARROWS_Y + 1) + y, new Pose2d(translation, rotation));
        }
      }
    }

    if (RobotBase.isSimulation()) {
      Pose2d[] arr = new Pose2d[0];

      DogLog.log("Repulsor/arrows", arrows.toArray(arr));
    }
  }

  Force getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Force();
    }
    var direction = displacement.getAngle();
    var mag = GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm() * displacement.getNorm()));
    return new Force(mag, direction);
  }

  Force getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Force.kZero;
    for (Obstacle obs : fixedObstacles) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Force getForce(Translation2d curLocation, Translation2d target) {
    var goalForce = getGoalForce(curLocation, target)
        .plus(getObstacleForce(curLocation, target));
    return goalForce;
  }

  public static SwerveSample sample(
      Translation2d trans, Rotation2d rot, double vx, double vy, double omega) {
    return new SwerveSample(
        0,
        trans.getX(),
        trans.getY(),
        rot.getRadians(),
        vx,
        vy,
        omega,
        0,
        0,
        0,
        new double[4],
        new double[4]);
  }

  public void setGoal(Translation2d goal) {
    this.goal = goal;
    updateArrows();
  }

  public SwerveSample getSample(
      Pose2d pose, double maxSpeed) {
    return getSample(pose, maxSpeed, pose.getRotation());
  }

  public SwerveSample getSample(
      Pose2d pose,
      double maxSpeed,
      Rotation2d goalRotation) {
    double stepSize_m;
    var curTrans = pose.getTranslation();
    var err = curTrans.minus(goal);
    var useGoal = false;
    DogLog.log("Repulsor/err", curTrans.getDistance(goal));
    double slowdownDist = 1;
    if (err.getNorm() < slowdownDist) { // slow down 1 meter out
      stepSize_m = MathUtil.interpolate(0, maxSpeed * 0.02, err.getNorm() / slowdownDist);
      useGoal = true;
    } else {
      stepSize_m = maxSpeed * 0.02;
    }

    var netForce = getForce(curTrans, goal);
    var step = new Translation2d(stepSize_m, netForce.getAngle());
    return sample(!useGoal ? curTrans.plus(step) : goal, goalRotation, (step.getX() / 0.02), (step.getY() / 0.02), 0);
  }
}