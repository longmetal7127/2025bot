package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.studica.frc.AHRS;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.configs.Swerve;
import frc.robot.configs.Swerve.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class DriveTrain extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);


  //sim zero timer
  private double bufferZeroTime = 0.25;

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]"); // TODO: figure out why this is a 4 and not 0 like in the docs
  private SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  // sim field
  private Field2d m_field = new Field2d();

  // Create and configure a drivetrain simulation configuration
  final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(GyroSimulation.getNav2X())
          
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(() ->  new SwerveModuleSimulation(
                DCMotor.getNeoVortex(1),
                DCMotor.getNeo550(1),
                Swerve.ModuleConstants.kDrivingMotorReduction,
                46.42,
                Amps.of(60),
                Amps.of(20),
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                Swerve.ModuleConstants.coefficientFriction
              )
            )
          
          // Configures the track length and track width (spacing between swerve modules)
          .withTrackLengthTrackWidth(Inches.of(Swerve.DriveConstants.kTrackWidth), Inches.of(Swerve.DriveConstants.kTrackWidth))
          
          // Configures the bumper size (dimensions of the robot bumper)
          .withBumperSize(Inches.of(35), Inches.of(35));

  public SwerveDriveSimulation swerveDriveSimulation;

  /** Creates a new DriveSubsystem. */
  public DriveTrain() {
    if(Robot.isSimulation()) {

      // create field on smart dashboard
      SmartDashboard.putData("Field", m_field);

      this.swerveDriveSimulation = new SwerveDriveSimulation(
          // Specify Configuration
          driveTrainSimulationConfig, 
          // Specify starting pose
          new Pose2d(3, 3, new Rotation2d())
      );
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation); 
      // creation the swerve simulation (please refer to previous documents)
      //resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI/2)));
      //angle.set(Math.PI/2);
      //zeroHeading();

      m_odometry.resetPose(
          new Pose2d(new Translation2d(0,0), new Rotation2d(Math.PI/2))
      );
    }
  }

  @Override
  public void periodic() {
    // sim case
    if (Robot.isSimulation()) {
      double dt = 0.02;

      // Update the odometry in the periodic block
      m_frontLeft.incrementSim(dt);
      m_frontRight.incrementSim(dt);
      m_rearLeft.incrementSim(dt);
      m_rearRight.incrementSim(dt);

      // get velocities of x, y, and rot
      ChassisSpeeds speeds = Swerve.DriveConstants.kDriveKinematics.toChassisSpeeds(
          new SwerveModuleState[] {
              m_frontLeft.getState(),
              m_frontRight.getState(),
              m_rearLeft.getState(),
              m_rearRight.getState()
          });
      m_odometry.resetPose(
          m_odometry.getPoseMeters().exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * dt,
                  speeds.vyMetersPerSecond * dt,
                  speeds.omegaRadiansPerSecond * dt)));

      // update robot pose
      m_field.setRobotPose(m_odometry.getPoseMeters());
      angle.set(-m_odometry.getPoseMeters().getRotation().getDegrees());

      if(bufferZeroTime <= 0) {
        zeroHeading();
        bufferZeroTime = 2;
      } else if(bufferZeroTime<1)
        bufferZeroTime -= dt;
    } else {
      // update odemetry normally
      m_odometry.update(
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          });
    }

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState() };
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
            m_rearRight.getPosition()
        },
        pose);
  }

  public Command cmdResetOdometry(Pose2d pose) {
    return this.run(() -> {
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

      SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  m_gyro.getRotation2d())
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

      
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
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
  public void zeroHeading() {
    m_gyro.reset();
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