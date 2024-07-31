package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.quixlib.math.MathUtils;
import org.littletonrobotics.junction.LoggedRobot;

public class ShotCalculator {
  public static class ShotInfo {
    public final double yaw;
    public final double yawVelocity;
    public double elevation;
    public final double elevationVelocity;
    public final double shotVelocity;
    public boolean feasibleShot;

    public ShotInfo(
        double yaw,
        double yawVelocity,
        double elevation,
        double elevationAngleVel,
        double shotVelocity,
        boolean feasibleShot) {
      this.yaw = yaw;
      this.yawVelocity = yawVelocity;
      this.elevation = elevation;
      this.elevationVelocity = elevationAngleVel;
      this.shotVelocity = shotVelocity;
      this.feasibleShot = feasibleShot;
    }
  }

  private static class StaticShotInfo {
    private final double yaw;
    private final double elevation;
    private final double shotVelocity;
    private final boolean feasibleShot;

    private StaticShotInfo(
        double yaw, double elevation, double shotVelocity, boolean feasibleShot) {
      this.yaw = yaw;
      this.elevation = elevation;
      this.shotVelocity = shotVelocity;
      this.feasibleShot = feasibleShot;
    }
  }

  public static ShotInfo computeSpeakerShotInfo(
      final double launchZOffset,
      final Translation3d goalPos,
      final Pose2d robotFieldPose,
      final Transform2d robotVel,
      final Transform2d robotAccel,
      final Transform2d robotFieldRelativeVel) {
    // Estimate velocities by finite differencing.
    final Twist2d dTwist =
        new Twist2d(
            robotVel.getX() * LoggedRobot.defaultPeriodSecs
                + 0.5
                    * robotAccel.getX()
                    * LoggedRobot.defaultPeriodSecs
                    * LoggedRobot.defaultPeriodSecs,
            robotVel.getY() * LoggedRobot.defaultPeriodSecs
                + 0.5
                    * robotAccel.getY()
                    * LoggedRobot.defaultPeriodSecs
                    * LoggedRobot.defaultPeriodSecs,
            robotVel.getRotation().getRadians() * LoggedRobot.defaultPeriodSecs
                + 0.5
                    * robotAccel.getRotation().getRadians()
                    * LoggedRobot.defaultPeriodSecs
                    * LoggedRobot.defaultPeriodSecs);
    final Pose2d nextRobotFieldPose = robotFieldPose.exp(dTwist);
    final StaticShotInfo elevationAndYawInfo =
        computeElevationAngleAndYaw(
            launchZOffset,
            goalPos,
            robotFieldPose.transformBy(Constants.Launcher.robotToLauncher),
            robotFieldRelativeVel);
    final StaticShotInfo nextElevationAndYawInfo =
        computeElevationAngleAndYaw(
            launchZOffset,
            goalPos,
            nextRobotFieldPose.transformBy(Constants.Launcher.robotToLauncher),
            robotFieldRelativeVel);

    final double yaw = elevationAndYawInfo.yaw;
    final double yawVelocity =
        (MathUtils.placeInScope(nextElevationAndYawInfo.yaw, yaw) - yaw)
            / LoggedRobot.defaultPeriodSecs;
    final double elevation = elevationAndYawInfo.elevation;
    final double elevationVelocity =
        (nextElevationAndYawInfo.elevation - elevation) / LoggedRobot.defaultPeriodSecs;
    final double shotVelocity = elevationAndYawInfo.shotVelocity;
    final boolean feasible =
        elevationAndYawInfo.feasibleShot && nextElevationAndYawInfo.feasibleShot;
    return new ShotInfo(yaw, yawVelocity, elevation, elevationVelocity, shotVelocity, feasible);
  }

  private static StaticShotInfo computeElevationAngleAndYaw(
      double launchZOffset,
      Translation3d goalPos,
      Pose2d launcherFieldPose,
      Transform2d robotFieldRelativeVel) {
    // TODO: Use launcherFieldRelativeVel instead of robotFieldRelativeVel
    final Translation2d goalPos2d = new Translation2d(goalPos.getX(), goalPos.getY());
    final Translation2d goalVector = goalPos2d.minus(launcherFieldPose.getTranslation());
    final double goalDistance = goalVector.getNorm();
    final double goalHeight = goalPos.getZ() - Constants.Launcher.launcherHeight - launchZOffset;

    double shotVelocity = Constants.Launcher.defaultShotVelocity;
    if (goalDistance < Constants.Launcher.scaleStartDistance) {
      shotVelocity = Constants.Launcher.minShotVelocity;
    } else if (goalDistance > Constants.Launcher.scaleEndDistance) {
      shotVelocity = Constants.Launcher.defaultShotVelocity;
    } else {
      final double slope =
          (Constants.Launcher.defaultShotVelocity - Constants.Launcher.minShotVelocity)
              / (Constants.Launcher.scaleEndDistance - Constants.Launcher.scaleStartDistance);
      shotVelocity =
          slope * (goalDistance - Constants.Launcher.scaleEndDistance)
              + Constants.Launcher.defaultShotVelocity;
    }

    // Binary search over elevationAngle for the best solution
    double lbElevation = 0.0;
    double ubElevation = Constants.Launcher.maxAngle;
    double elevationAngle = 0.0;
    double yaw = 0.0;
    boolean feasibleShot = false;
    for (int i = 0; i < 10; i++) { // 10 iterations for elevationAngle to converge
      elevationAngle = (lbElevation + ubElevation) * 0.5;
      final double shotHorizontalVel = shotVelocity * Math.cos(elevationAngle);

      // Binary search to find the yaw that results in the total shot vel vector lining up with the
      // goal vector.
      // TODO: Can we do this without binary searching?
      Translation2d shotVelVector = new Translation2d();
      double lbYaw = launcherFieldPose.getRotation().getRadians() - Math.PI;
      double ubYaw = launcherFieldPose.getRotation().getRadians() + Math.PI;
      for (int j = 0; j < 16; j++) { // 16 iterations for yaw to converge
        yaw = (lbYaw + ubYaw) * 0.5;
        shotVelVector =
            new Translation2d(shotHorizontalVel * Math.cos(yaw), shotHorizontalVel * Math.sin(yaw));
        final Translation2d totalShotVelVector =
            robotFieldRelativeVel.getTranslation().plus(shotVelVector);
        if (totalShotVelVector.getAngle().minus(goalVector.getAngle()).getRadians() > 0.0) {
          ubYaw = yaw;
        } else {
          lbYaw = yaw;
        }
      }

      final Translation2d totalShotVelVector =
          robotFieldRelativeVel.getTranslation().plus(shotVelVector);
      final double totalShotHorizontalVel = totalShotVelVector.getNorm();
      final double totalShotVerticalVel = shotVelocity * Math.sin(elevationAngle);

      // Calculate height of the shot at the goal to see if we are above or below it with the given
      // elevation angle.
      final double t = goalDistance / totalShotHorizontalVel;
      final double shotHeightAtGoal = (totalShotVerticalVel * t) - (0.5 * Constants.g * t * t);
      if (shotHeightAtGoal > goalHeight) {
        ubElevation = elevationAngle;
      } else {
        lbElevation = elevationAngle;
      }

      // Only consider this a feasible shot if the piece hits the speaker while its rising
      feasibleShot = (totalShotVerticalVel - Constants.g * t) > 0;
    }
    return new StaticShotInfo(yaw, elevationAngle, shotVelocity, feasibleShot);
  }

  public static ShotInfo computeFeedShotInfo(
      final Translation3d goalPos,
      final Pose2d robotFieldPose,
      final Transform2d robotVel,
      final Transform2d robotAccel,
      final Transform2d robotFieldRelativeVel) {
    // Estimate velocities by finite differencing.
    final Twist2d dTwist =
        new Twist2d(
            robotVel.getX() * LoggedRobot.defaultPeriodSecs
                + 0.5
                    * robotAccel.getX()
                    * LoggedRobot.defaultPeriodSecs
                    * LoggedRobot.defaultPeriodSecs,
            robotVel.getY() * LoggedRobot.defaultPeriodSecs
                + 0.5
                    * robotAccel.getY()
                    * LoggedRobot.defaultPeriodSecs
                    * LoggedRobot.defaultPeriodSecs,
            robotVel.getRotation().getRadians() * LoggedRobot.defaultPeriodSecs
                + 0.5
                    * robotAccel.getRotation().getRadians()
                    * LoggedRobot.defaultPeriodSecs
                    * LoggedRobot.defaultPeriodSecs);
    final Pose2d nextRobotFieldPose = robotFieldPose.exp(dTwist);
    final StaticShotInfo shotInfo =
        computeFeedStaticShotInfo(
            goalPos,
            robotFieldPose.transformBy(Constants.Launcher.robotToLauncher),
            robotFieldRelativeVel);
    final StaticShotInfo nextShotInfo =
        computeFeedStaticShotInfo(
            goalPos,
            nextRobotFieldPose.transformBy(Constants.Launcher.robotToLauncher),
            robotFieldRelativeVel);

    final double yaw = shotInfo.yaw;
    final double yawVelocity =
        (MathUtils.placeInScope(nextShotInfo.yaw, yaw) - yaw) / LoggedRobot.defaultPeriodSecs;
    final double elevation = shotInfo.elevation;
    final double elevationVelocity =
        (nextShotInfo.elevation - elevation) / LoggedRobot.defaultPeriodSecs;
    final double shotVelocity = shotInfo.shotVelocity;
    final boolean feasible = shotInfo.feasibleShot && nextShotInfo.feasibleShot;
    return new ShotInfo(yaw, yawVelocity, elevation, elevationVelocity, shotVelocity, feasible);
  }

  private static StaticShotInfo computeFeedStaticShotInfo(
      Translation3d goalPos, Pose2d launcherFieldPose, Transform2d robotFieldRelativeVel) {
    // TODO: Use launcherFieldRelativeVel instead of robotFieldRelativeVel
    final Translation2d goalPos2d = new Translation2d(goalPos.getX(), goalPos.getY());
    final Translation2d goalVector = goalPos2d.minus(launcherFieldPose.getTranslation());
    final double goalDistance = goalVector.getNorm();
    final double goalHeight = goalPos.getZ() - Constants.Launcher.launcherHeight;

    // Binary search over elevationAngle for the best solution
    double lbElevation = 0.0;
    double ubElevation = Constants.Launcher.maxAngle;
    double elevationAngle = 0.0;
    double yaw = 0.0;
    boolean feasibleShot = false;
    double shotVelocity = 0.0;
    for (int i = 0; i < 10; i++) { // 10 iterations for elevationAngle to converge
      elevationAngle = (lbElevation + ubElevation) * 0.5;

      // Determine the shot velocity that peaks at feedShotApexHeight.
      final double totalShotVerticalVel =
          Math.sqrt(
              2.0
                  * Constants.g
                  * (Constants.FieldPoses.feedShotApexHeight - Constants.Launcher.launcherHeight));
      // Shoot slightly harder than computed.
      final double kFudgeFactor = 1.2;
      shotVelocity = totalShotVerticalVel / Math.sin(elevationAngle) * kFudgeFactor;

      final double shotHorizontalVel = shotVelocity * Math.cos(elevationAngle);

      // Binary search to find the yaw that results in the total shot vel vector lining up with the
      // goal vector.
      // TODO: Can we do this without binary searching?
      Translation2d shotVelVector = new Translation2d();
      double lbYaw = launcherFieldPose.getRotation().getRadians() - Math.PI;
      double ubYaw = launcherFieldPose.getRotation().getRadians() + Math.PI;
      for (int j = 0; j < 16; j++) { // 16 iterations for yaw to converge
        yaw = (lbYaw + ubYaw) * 0.5;
        shotVelVector =
            new Translation2d(shotHorizontalVel * Math.cos(yaw), shotHorizontalVel * Math.sin(yaw));
        final Translation2d totalShotVelVector =
            robotFieldRelativeVel.getTranslation().plus(shotVelVector);
        if (totalShotVelVector.getAngle().minus(goalVector.getAngle()).getRadians() > 0.0) {
          ubYaw = yaw;
        } else {
          lbYaw = yaw;
        }
      }

      final Translation2d totalShotVelVector =
          robotFieldRelativeVel.getTranslation().plus(shotVelVector);
      final double totalShotHorizontalVel = totalShotVelVector.getNorm();

      // Check whether the downward part of the trajectory at the goal height is closer or further
      // than the goal.
      // goalHeight = launchHeight + totalShotVerticalVel * t + 0.5 * g * t * t
      final double t =
          (totalShotVerticalVel
                  + Math.sqrt(
                      totalShotVerticalVel * totalShotVerticalVel
                          + 2.0 * Constants.g * (Constants.Launcher.launcherHeight - goalHeight)))
              / Constants.g;
      final double shotDistance = totalShotHorizontalVel * t;
      if (shotDistance < goalDistance) {
        ubElevation = elevationAngle;
      } else {
        lbElevation = elevationAngle;
      }

      // Only consider this a feasible shot if the shoth velocity is between 5 and 20 m/s.
      feasibleShot = shotVelocity >= 5.0 && shotVelocity <= 20.0;
    }
    // Shots curve left, so aim more right.
    yaw += Math.toRadians(-5.0);
    return new StaticShotInfo(yaw, elevationAngle, shotVelocity, feasibleShot);
  }
}
