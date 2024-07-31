package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.swerve.QuikPlanSwervePartialTrajectoryReader;
import java.util.ArrayList;

public interface AutoCommand {
  public Command getCommand();

  public Pose2d getInitialPose();

  public ArrayList<QuikPlanSwervePartialTrajectoryReader> getPartialTrajectories();

  public default void loadAndUpdateViz(final Field2d fieldViz) {
    final double kNumSamples = 100;
    final var vizPoses = new ArrayList<Pose2d>();
    for (var traj : getPartialTrajectories()) {
      traj.loadSelectedFile();
      for (int i = 0; i < kNumSamples; i++) {
        double t = traj.getTotalTime() * i / kNumSamples;
        vizPoses.add(traj.getState(t).pose);
      }
    }
    fieldViz.getObject("traj").setPoses(vizPoses);
    LoggerHelper.recordPose2dList("AutoTraj", vizPoses);
  }
}
