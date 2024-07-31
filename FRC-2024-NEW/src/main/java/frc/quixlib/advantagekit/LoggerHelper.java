package frc.quixlib.advantagekit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class LoggerHelper {
  public static void recordCurrentCommand(String prefix, SubsystemBase subsystem) {
    final var currentCommand = subsystem.getCurrentCommand();
    Logger.recordOutput(
        prefix + "/Current Command", currentCommand == null ? "None" : currentCommand.getName());
  }

  public static void recordPose2dList(String key, ArrayList<Pose2d> list) {
    Pose2d[] array = new Pose2d[list.size()];
    list.toArray(array);
    Logger.recordOutput(key, array);
  }
}
