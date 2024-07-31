package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.swerve.QuixSwerveController;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PipelineConfig;

public class Constants {
  public static final String kCanivoreName = "canivore";
  public static final double g = 9.81; // m/s/s

  public static final class IMU {
    public static final CANDeviceID pigeonID = new CANDeviceID(0, kCanivoreName);
    public static final double gyroTrimZ =
        (62.235614 - (20.0 * Math.PI)) / 10.0; // rad per rotation
  }

  public static final class Cameras { // TODO: Update with real values
    public static final class LeftCam {
      public static final double fovWidth = Math.toRadians(70); // rad
      public static final double fovHeight = Math.toRadians(47.5); // rad
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(10.68),
                  Units.inchesToMeters(12.25),
                  Units.inchesToMeters(23.38)),
              new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(
                Fiducial.Type.APRILTAG,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    916.965230021908,
                    0.0,
                    661.6928938560056,
                    0.0,
                    916.8276406094255,
                    435.5533504564346,
                    0.0,
                    0.0,
                    1.0),
                MatBuilder.fill(
                    Nat.N5(),
                    Nat.N1(),
                    0.05009856981900392,
                    -0.07369910749297018,
                    -1.0660525417228317E-5,
                    1.3422933851637837E-4,
                    0.009667561013012865)),
          };
    }

    public static final class RightCam {
      public static final double fovWidth = Math.toRadians(70); // rad
      public static final double fovHeight = Math.toRadians(47.5); // rad
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(10.68),
                  Units.inchesToMeters(-12.25),
                  Units.inchesToMeters(23.38)),
              new Rotation3d(0.0, Units.degreesToRadians(-24.5), 0.0)); // Manually tuned extrinsics
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(
                Fiducial.Type.APRILTAG,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    903.7449893954214,
                    0.0,
                    654.6877054577706,
                    0.0,
                    903.9577323243168,
                    369.13545590011745,
                    0.0,
                    0.0,
                    1.0),
                MatBuilder.fill(
                    Nat.N5(),
                    Nat.N1(),
                    0.05318440103944059,
                    -0.07489968261371575,
                    -5.477461690531807E-4,
                    -7.032312933604217E-4,
                    0.009722692505020142)),
          };
    }

    public final class BackLeft {
      public static final double fovWidth = Math.toRadians(70); // rad
      public static final double fovHeight = Math.toRadians(47.5); // rad
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(5.55),
                  Units.inchesToMeters(12.25),
                  Units.inchesToMeters(12.27)),
              new Rotation3d(0.0, Units.degreesToRadians(22.5), Math.PI));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(
                Fiducial.Type.NOTE,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    916.965230021908,
                    0.0,
                    661.6928938560056,
                    0.0,
                    916.8276406094255,
                    435.5533504564346,
                    0.0,
                    0.0,
                    1.0),
                MatBuilder.fill(
                    Nat.N5(),
                    Nat.N1(),
                    0.05009856981900392,
                    -0.07369910749297018,
                    -1.0660525417228317E-5,
                    1.3422933851637837E-4,
                    0.009667561013012865)),
          };
    }

    public static final class BackRight {
      public static final double fovWidth = Math.toRadians(70); // rad
      public static final double fovHeight = Math.toRadians(47.5); // rad
      public static final Transform3d robotToCameraT =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(5.55),
                  Units.inchesToMeters(-12.25),
                  Units.inchesToMeters(12.27)),
              new Rotation3d(0.0, Units.degreesToRadians(22.5), Math.PI));
      public static final PipelineConfig[] pipelineConfigs =
          new PipelineConfig[] {
            new PipelineConfig(
                Fiducial.Type.NOTE,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    903.7449893954214,
                    0.0,
                    654.6877054577706,
                    0.0,
                    903.9577323243168,
                    369.13545590011745,
                    0.0,
                    0.0,
                    1.0),
                MatBuilder.fill(
                    Nat.N5(),
                    Nat.N1(),
                    0.05318440103944059,
                    -0.07489968261371575,
                    -5.477461690531807E-4,
                    -7.032312933604217E-4,
                    0.009722692505020142)),
          };
    }
  }

  public static final class Swerve { // TODO: updated with real values
    public static final double maxDriveSpeed = 5.0; // m/s
    public static final double maxDriveAcceleration = 8.0; // m/s/s
    public static final double maxAngularVelocity = Math.PI * 2.0; // rad/s
    public static final double maxAngularAcceleration = Math.PI * 4.0; // rad/s/s
    public static final double trackWidth = Units.inchesToMeters(24.25);
    public static final double wheelBase = Units.inchesToMeters(24.25);
    public static final double wheelDiameter = Units.inchesToMeters(3.91);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final MechanismRatio driveRatio =
        new MechanismRatio(1.0, 5.51, wheelCircumference);
    public static final MechanismRatio steeringRatio = new MechanismRatio(1.0, 468.0 / 35.0);
    public static final double steerDriveCouplingRatio = 42.0 / 10.0;
    public static final PIDConfig driveVelocityPIDConfig = new PIDConfig(0.2, 0.0, 0.0);
    public static final SimpleMotorFeedforward driveVelocityFeedforward =
        new SimpleMotorFeedforward(0.2, 3.8, 0.0); // TODO: is this too high?
    public static final PIDConfig steeringPIDConfig = new PIDConfig(8.0, 0.0, 0.0);

    /* Swerve Teleop Slew Rate Limits */
    public static final double linearSlewRate = 20.0; // m/s/s
    public static final double angularSlewRate = 40.0; // rad/s/s
    public static final double stickDeadband = 0.06;

    /* Module Slew Rate Limits */
    // Max module acceleration can be high since drive wheels can be backdriven.
    public static final double maxModuleAcceleration = 20.0; // m/s/s
    public static final double maxModuleSteeringRate = 8.0 * Math.PI; // rad/s/s

    /* Allowable scrub */
    public static final double autoScrubLimit = 0.25; // m/s
    public static final double teleopScrubLimit = 0.25; // m/s

    public static final QuixSwerveController driveController =
        new QuixSwerveController(
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.1));

    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(0, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(1, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(0, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 1.537049;
    }

    /* Rear Left Module - Module 1 */
    public static final class RearLeft {
      public static final CANDeviceID driveMotorID = new CANDeviceID(2, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(3, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(1, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 1.613748;
    }

    /* Rear Right Module - Module 2 */
    public static final class RearRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(4, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(5, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(2, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 3.472933;
    }

    /* Front Right Module - Module 3 */
    public static final class FrontRight {
      public static final CANDeviceID driveMotorID = new CANDeviceID(6, kCanivoreName);
      public static final CANDeviceID steeringMotorID = new CANDeviceID(7, kCanivoreName);
      public static final CANDeviceID canCoderID = new CANDeviceID(3, kCanivoreName);
      public static final Translation2d modulePosition =
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
      public static final double absEncoderOffsetRad = 0.581379;
    }
  }

  public static final class Intake {
    public static final int beamBreakPort = 1;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(9, kCanivoreName);
    public static final MechanismRatio rollerMotorRatio =
        new MechanismRatio(1, (36.0 / 12.0) * (36.0 / 28.0));
    public static final boolean rollerMotorInvert = false;
    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.3, 0.12, 0);
    public static final PIDConfig rollerPIDConfig = new PIDConfig(0.1, 0, 0);
    public static final int rollerVelocitySlot = 0;

    public static final CANDeviceID deployMotorID = new CANDeviceID(8, kCanivoreName);
    public static final MechanismRatio deployMotorRatio =
        new MechanismRatio(1, (42.0 / 10.0) * (42.0 / 22.0) * (42.0 / 16.0) * (36.0 / 16.0));
    public static final boolean deployMotorInvert = true;
    public static final PIDConfig deployPIDConfig = new PIDConfig(2.0, 0, 0.3, 0, 0.12, 0.007, 0);
    public static final int deployPositionSlot = 0;
    public static final double deployMaxVelocity = 12.0; // rad/s
    public static final double deployMaxAcceleration = 140.0; // rad/s^2
    public static final double deployMaxJerk = 800.0; // rad/s^3

    public static final double bootAbsPositionOffset = Units.degreesToRadians(1.8);
    public static final double minAngle = Units.degreesToRadians(-50.0); // rads
    public static final double maxAngle = Units.degreesToRadians(90.0); // rads
    public static final double startingAngle = maxAngle + bootAbsPositionOffset;
    public static final double intakeDeployAngle = Math.toRadians(-50); // rad
    public static final double intakeStowAngle = Math.toRadians(80); // rad
    public static final double intakeRollerVelocity = 100; // rad/s

    // For simulation.
    public static final double simArmMOI = 0.2; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(10.0); // m
    public static final double simRollerMOI = 0.01; // kgMetersSquared
  }

  public static final class Elevator {
    public static final CANDeviceID motorID = new CANDeviceID(10, kCanivoreName);
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.273); // 16T #25
    public static final MechanismRatio motorRatio =
        new MechanismRatio(
            1, (28.0 / 10.0) * (38.0 / 16.0) * (42.0 / 18.0), Math.PI * sprocketPitchDiameter);
    public static final boolean motorInvert = false;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = new PIDConfig(5, 0, 0.1, 0, 0.12, 0, 0.4);
    public static final double maxVelocity = 2.0; // m/s
    public static final double maxAcceleration = 30.0; // m/s^2
    public static final double maxJerk = 0.0; // m/s^3 (0 disables jerk limit)

    // TODO: use real numbers
    public static final double minHeight = 0.0; // m
    public static final double powerCutoffHeight = Units.inchesToMeters(0.5); // m
    public static final double maxHeight = Units.inchesToMeters(16.0); // m
    public static final double stowHeight = Units.inchesToMeters(0.125); // m
    public static final double stowTolerance = Units.inchesToMeters(1.0); // m
    public static final double scoreAmpHeight = Units.inchesToMeters(8.0); // m
    public static final double scoreAmpTolerance = Units.inchesToMeters(1.0); // m
    public static final double climbRetractHeight = Units.inchesToMeters(0.0); // m
    public static final double climbExtendHeight = Units.inchesToMeters(16.0); // m

    // For simulation.
    public static final double simCarriageMass = 15.0; // kg

    // TODO: find real values
    public static final Constraints elevatorTrapConstraints =
        new Constraints(1, 3); // m/s and m/s^2
    public static final ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(0.0, 0.0, 0.0); // new ElevatorFeedforward(0.35, 0.15, 15.8);
  }

  public static final class Climber {
    public static final CANDeviceID motorID = new CANDeviceID(11, kCanivoreName);
    public static final MechanismRatio motorRatio =
        new MechanismRatio(1, (40.0 / 14.0) * (32.0 / 10.0));
    public static final boolean motorInvert = false;
    public static final int unloadedPositionSlot = 0;
    public static final PIDConfig unloadedPIDConfig = new PIDConfig(1, 0, 0, 0, 0.12, 0, 0.0);
    public static final int loadedPositionSlot = 1;
    public static final PIDConfig loadedPIDConfig = new PIDConfig(1, 0, 0, 0, 0.12, 0, 0.0);
    public static final double maxVelocity = 16.0 * Math.PI; // rad/s
    public static final double maxAcceleration = 16.0 * Math.PI; // rad/s^2
    public static final double maxJerk = 0.0; // rad/s^3 (0 disables jerk limit)
    public static final double slowMaxVelocity = 12.0 * Math.PI; // rad/s
    public static final double slowMaxAcceleration = 16.0 * Math.PI; // rad/s^2
    public static final double slowMaxJerk = 0.0; // rad/s^3 (0 disables jerk limit)

    public static final double minPos = 0.0;
    public static final double partialGrabPos = 40.0; // rads
    public static final double extendPos = 57.0; // rads
  }

  public static final class Launcher {
    public static final int beamBreakPort = 0;

    public static final CANDeviceID armMotorID = new CANDeviceID(12, kCanivoreName);
    public static final MechanismRatio armMotorRatio =
        new MechanismRatio(1, (28.0 / 10.0) * (60.0 / 14.0) * (60.0 / 16.0));
    public static final boolean armMotorInvert = true;

    public static final CANDeviceID redirectMotorID = new CANDeviceID(13, kCanivoreName);
    public static final MechanismRatio redirectMotorRatio = new MechanismRatio(12, 18);
    public static final boolean redirectMotorInvert = true;

    public static final CANDeviceID feedMotorID = new CANDeviceID(14, kCanivoreName);
    public static final MechanismRatio feedMotorRatio = new MechanismRatio(16, 24);
    public static final boolean feedMotorInvert = true;

    public static final CANDeviceID lowerMotorID = new CANDeviceID(15, kCanivoreName);
    public static final MechanismRatio lowerMotorRatio = new MechanismRatio(1, 1);
    public static final boolean lowerMotorInvert = false;

    public static final CANDeviceID upperMotorID = new CANDeviceID(16, kCanivoreName);
    public static final MechanismRatio upperMotorRatio = new MechanismRatio(1, 1);
    public static final boolean upperMotorInvert = false;

    public static final ArmFeedforward armFeedForward = new ArmFeedforward(0.0, 0.3, 0.6);
    public static final Constraints armTrapConstraints =
        new Constraints(12.5, 80.0); // rad/s and rad/s^2
    public static final Constraints armSlowTrapConstraints =
        new Constraints(6.0, 80.0); // rad/s and rad/s^2
    public static final int armPositionPIDSlot = 0;
    public static final PIDConfig armPositionPIDConfig = new PIDConfig(3.0, 0.0, 0.01);

    public static final SimpleMotorFeedforward redirectRollerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.03);
    public static final int redirectVelocityPIDSlot = 0;
    public static final PIDConfig redirectVelocityPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final int redirectPositionPIDSlot = 1;
    public static final PIDConfig redirectPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    public static final SimpleMotorFeedforward feedRollerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.028);
    public static final int feedVelocityPIDSlot = 0;
    public static final PIDConfig feedVelocityPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final int feedPositionPIDSlot = 1;
    public static final PIDConfig feedPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    public static final SimpleMotorFeedforward launcherFeedforward =
        new SimpleMotorFeedforward(0.0, 0.019);
    public static final int launcherVelocityPIDSlot = 0;
    public static final PIDConfig launcherVelocityPIDConfig = new PIDConfig(0.2, 0.0, 0.0);

    // TODO: Use real values
    public static final double bootAbsPositionOffset = Units.degreesToRadians(-1.6);
    public static final double minAngle = Units.degreesToRadians(-128.0); // rads (trap position)
    public static final double maxAngle = Units.degreesToRadians(55.0); // rads (stow position)
    public static final double startingAngle = maxAngle + bootAbsPositionOffset;
    public static final double cgOffset = Units.degreesToRadians(30.0);

    public static final double climbAngle = Units.degreesToRadians(-45.0); // rads (trap position)
    public static final double trapAngle = Units.degreesToRadians(-128.0); // rads (trap position)

    public static final double intakeAngle = Units.degreesToRadians(45);
    public static final double intakeAngleTolerance = Units.degreesToRadians(5);

    public static final double subwooferLaunchAngle = Units.degreesToRadians(55);
    public static final double podiumLaunchAngle = Units.degreesToRadians(30);
    public static final double feedLaunchAngle = Units.degreesToRadians(45);
    public static final double launchAngleTolerance = Units.degreesToRadians(2);
    public static final double launchVelocity = 550.0; // rads/s
    public static final double launchFeedShotVelocity = 310.0; // rads/s
    public static final double launchVelocityTolerance = 50.0; // rads/s
    public static final double autoLaunchStartVelocity = 300.0; // rads/s

    public static final double scoreAmpArmAngle = Units.degreesToRadians(-105.0); // rads
    public static final double scoreAmpArmAngleTolerance = Units.degreesToRadians(2); // rads

    public static final double intakeFromSourceAngle = Units.degreesToRadians(55); // rads
    public static final double intakeFromSourceLaunchVelocity = -100; // rads/s
    public static final double intakeFromSourceFeedPower = -0.25;

    public static final double intakeFeedVelocity = 70; // rad/s
    public static final double scoreAmpFeedVelocity = 300; // rad/s
    public static final double scoreSpeakerFeedVelocity = 300; // rad/s
    public static final double scoreTrapFeedVelocity = 300; // rad/s
    public static final double scoreTrapReverseVelocity = 50; // rad/s

    public static final double shotVelocity = 20.0; // m/s
    public static final Transform2d robotToLauncher =
        new Transform2d(Units.inchesToMeters(12.0), 0.0, new Rotation2d());
    public static final double launcherHeight = Units.inchesToMeters(21);

    public static final double rollerBeamBreakOffset = 1.5 * Math.PI; // rads

    // For simulation.
    public static final double simArmMOI = 0.3; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(4.0); // m
    public static final double simWheelMOI = 0.01; // kgMetersSquared
    public static final double simRollerMOI = 0.003; // kgMetersSquared
  }

  // Placeholder values
  public static final class Example {
    public static final CANDeviceID motorID = new CANDeviceID(99, kCanivoreName);
    public static final MechanismRatio motorRatio = new MechanismRatio(1, 1);
    public static final boolean motorInvert = false;
  }

  public static final class Viz {
    public static final double xOffset = Units.inchesToMeters(12.0);
    public static final double intakePivotX = xOffset + Units.inchesToMeters(27.25);
    public static final double intakePivotY = Units.inchesToMeters(11.25);
    public static final double intakeArmLength = Units.inchesToMeters(14.0);
    public static final double elevatorBaseX = xOffset + Units.inchesToMeters(12.0);
    public static final double elevatorBaseY = Units.inchesToMeters(3.0);
    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(105.0);
    public static final double elevatorBaseLength = Units.inchesToMeters(24.0);
    public static final double elevatorCarriageLength = Units.inchesToMeters(24.0);
    public static final double launcherArmPivotX = Units.inchesToMeters(17.0);
    public static final double launcherArmLength = Units.inchesToMeters(15.0);
    public static final double launcherWheelX = Units.inchesToMeters(-3.0);
    public static final double launcherTopWheelY = Units.inchesToMeters(4.625);
    public static final double launcherBottomWheelY = Units.inchesToMeters(-0.625);
    public static final double launcherFeedRollerX = Units.inchesToMeters(12.0);
    public static final double launcherFeedRollerY = Units.inchesToMeters(1.875);
    public static final double launcherRedirectRollerX = Units.inchesToMeters(12.0);
    public static final double launcherRedirectRollerY = Units.inchesToMeters(5.125);

    public static final double angularVelocityScalar = 0.01;
  }

  public static final class Viz3d {
    public static final Pose3d intakePivotBase =
        new Pose3d(Units.inchesToMeters(-12.5), 0.0, Units.inchesToMeters(11.0), new Rotation3d());
    public static final Pose3d elevatorBase =
        new Pose3d(
            Units.inchesToMeters(3.0),
            0,
            Units.inchesToMeters(2.75),
            new Rotation3d(0, Math.toRadians(15.0), 0));
    public static final Transform3d elevatorCarriageToLauncherArmPivot =
        new Transform3d(0, 0, Units.inchesToMeters(16.0), new Rotation3d());
    public static final Pose3d climberPivot =
        elevatorBase.transformBy(
            new Transform3d(
                0, 0, Units.inchesToMeters(15.0), new Rotation3d(0, Math.toRadians(-30), 0)));
  }

  public static final class FieldPoses {
    public static final double midline = Units.inchesToMeters(325.61);
    public static final double blueWingLine = Units.inchesToMeters(231.20);
    public static final double feedShotApexHeight = Units.inchesToMeters(120);
    public static final Transform3d speakerCenterTagToGoalOffset =
        new Transform3d(
            Units.inchesToMeters(6),
            Units.inchesToMeters(0),
            Units.inchesToMeters(20 + 2),
            new Rotation3d());
    public static final Transform3d ampTagToFeedShotOffset =
        new Transform3d(Units.inchesToMeters(36), 0.0, 0.0, new Rotation3d());
    public static final double speakerGoalWidth = Units.inchesToMeters(40.0);
    public static final Transform2d ampApproachOffsetTransform =
        new Transform2d(new Translation2d(1.0, 0), new Rotation2d(Math.toRadians(180)));
    public static final Transform2d ampScoreOffsetTransform =
        new Transform2d(new Translation2d(0.7, 0), new Rotation2d(Math.toRadians(180)));
  }
}
