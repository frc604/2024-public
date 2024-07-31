package frc.quixlib.viz;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Stack;

public class Viz2d extends SubsystemBase {
  private final ArrayList<Link2d> m_children = new ArrayList<>();

  private final double m_pixelsPerMeter;
  protected final Mechanism2d m_viz;

  public Viz2d(String name, double width, double height, double pixelsPerMeter) {
    m_pixelsPerMeter = pixelsPerMeter;
    m_viz = new Mechanism2d(metersToPixels(width), metersToPixels(height));
    SmartDashboard.putData(name, m_viz);
  }

  /** Adds a top-level link. */
  public Link2d addLink(Link2d link) {
    m_children.add(link);
    return link;
  }

  protected double metersToPixels(double meters) {
    return meters * m_pixelsPerMeter;
  }

  @Override
  public void simulationPeriodic() {
    // DFS through the tree of links and update viz for each one.
    Stack<Pair<Link2d, Transform2d>> stack = new Stack<>();
    for (Link2d child : m_children) {
      stack.push(new Pair<>(child, new Transform2d()));
    }
    while (!stack.empty()) {
      var linkAndTransform = stack.pop();
      Link2d link = linkAndTransform.getFirst();
      Transform2d transform = linkAndTransform.getSecond();
      link.draw(this, m_pixelsPerMeter, transform);
      for (var child : link.getChildren()) {
        stack.push(new Pair<>(child, transform.plus(link.getRelativeTransform())));
      }
    }
  }
}
