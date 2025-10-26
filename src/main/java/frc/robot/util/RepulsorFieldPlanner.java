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

    protected double distToForceMag(double dist, double maxRange) {
      if (Math.abs(dist) > maxRange) {
        return 0;
      }
      if (MathUtil.isNear(0, dist, 1e-2)) {
        dist = 1e-2;
      }
      var forceMag = strength / (dist * dist);
      forceMag -= strength / (maxRange * maxRange);
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double radius = 0.5;
    double maxRange = 4.0;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > maxRange) {
        return new Force();
      }
      var outwardsMag = distToForceMag(loc.getDistance(position) - radius, maxRange - radius);
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
    double maxRange = 4.0;

    public SnowmanObstacle(Translation2d loc, double strength, double radius, boolean positive) {
      super(strength, positive);
      this.loc = loc;
      this.radius = radius;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > maxRange) {
        return new Force();
      }
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      // 1 meter away from loc, opposite target.
      var sidewaysCircle = new Translation2d(1, targetToLoc.getAngle()).plus(loc);
      var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position), maxRange);
      var outwardsMag = distToForceMag(Math.max(0.01, loc.getDistance(position) - radius), maxRange - radius);
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
    double maxRange;

    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getY() - y);
      if (dist > maxRange) {
        return new Force();
      }
      return new Force(0, distToForceMag(y - position.getY(), maxRange));
    }
  }

  static class VerticalObstacle extends Obstacle {
    double x;
    double maxRange;

    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    public Force getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getX() - x);
      if (dist > maxRange) {
        return new Force();
      }
      return new Force(distToForceMag(x - position.getX(), maxRange), 0);
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
      new TeardropObstacle(Constants.BLUE_REEF, 1, 3, .85, 3, 2),
      new TeardropObstacle(Constants.RED_REEF, 1, 3, .85, 3, 2));

  public static final List<Obstacle> WALLS = List.of(
      new HorizontalObstacle(0.0, 0.5, .5, true),
      new HorizontalObstacle(Constants.FIELD_WIDTH_METERS, 0.5, .5, false),
      new VerticalObstacle(0.0, 0.5, .5, true),
      new VerticalObstacle(Constants.FIELD_LENGTH_METERS, 0.5, .5, false));

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

    DogLog.log("Repulsor/err", curTrans.getDistance(goal));
    double slowdownDist = 0.75;
    if (err.getNorm() < slowdownDist) { // slow down 1 meter out
      stepSize_m = MathUtil.interpolate(0, maxSpeed * 0.02, err.getNorm() / slowdownDist);
    } else {
      stepSize_m = maxSpeed * 0.02;
    }

    var netForce = getForce(curTrans, goal);
    var step = new Translation2d(stepSize_m, netForce.getAngle());
    return sample(curTrans.plus(step), goalRotation, (step.getX() / 0.02), (step.getY() / 0.02), 0);
  }
}