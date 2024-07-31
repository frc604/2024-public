package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.ShotCalculator.ShotInfo;
import org.junit.jupiter.api.*;

public class ShotCalculatorTest {
  private static final double kTolDeg = 1.0;
  private static final Translation3d kGoalPos =
      Fiducials.aprilTagFiducials[6] // Blue center speaker tag
          .getPose()
          .plus(Constants.FieldPoses.speakerCenterTagToGoalOffset)
          .getTranslation();
  private static final double goalYPos = kGoalPos.getY();

  private void checkStaticShot(double xPosInches, double elevationDegrees) {
    final ShotInfo shotInfo =
        ShotCalculator.computeSpeakerShotInfo(
            0.0,
            kGoalPos,
            new Pose2d(
                new Translation2d(Units.inchesToMeters(xPosInches), goalYPos),
                new Rotation2d(Math.PI)),
            new Transform2d(),
            new Transform2d(),
            new Transform2d());
    assertEquals(elevationDegrees, Math.toDegrees(shotInfo.elevation), kTolDeg);
    assertEquals(0.0, shotInfo.elevationVelocity, kTolDeg);
    assertEquals(Math.PI, shotInfo.yaw, kTolDeg);
    assertEquals(0.0, shotInfo.yawVelocity, kTolDeg);

    final double computedShotHeightAtGoal =
        computeStaticShotHeightAtGoalInches(xPosInches, elevationDegrees);
    final double expectedShotHeightAtGoal =
        computeStaticShotHeightAtGoalInches(xPosInches, Math.toDegrees(shotInfo.elevation));
    assertEquals(expectedShotHeightAtGoal, computedShotHeightAtGoal, 4.0);
  }

  private double computeStaticShotHeightAtGoalInches(double xPosInches, double elevationDegrees) {
    final double horizontalVel =
        Constants.Launcher.shotVelocity * Math.cos(Math.toRadians(elevationDegrees));
    final double verticalVel =
        Constants.Launcher.shotVelocity * Math.sin(Math.toRadians(elevationDegrees));
    final double t =
        (Units.inchesToMeters(xPosInches)
                - Constants.Launcher.robotToLauncher.getX()
                - Constants.FieldPoses.speakerCenterTagToGoalOffset.getX())
            / horizontalVel;
    final double shotHeightAtGoal = (verticalVel * t) - (0.5 * Constants.g * t * t);
    return Units.metersToInches(shotHeightAtGoal + Constants.Launcher.launcherHeight);
  }

  @Test
  public void testStationary() {
    // checkStaticShot(63.0, 55.0);
    // checkStaticShot(99.0, 35.0);
    checkStaticShot(135.0, 28.0);
    checkStaticShot(171.0, 23.0);
    checkStaticShot(207.0, 20.0);
    checkStaticShot(243.0, 18.0);
    checkStaticShot(279.0, 16.5);
    checkStaticShot(315.0, 16.0);
  }
}
