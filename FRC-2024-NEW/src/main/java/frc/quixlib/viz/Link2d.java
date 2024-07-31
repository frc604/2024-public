package frc.quixlib.viz;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Link2d {
  private final ArrayList<Link2d> m_children = new ArrayList<>();

  private final LoggedMechanismRoot2d m_root;
  private final LoggedMechanismLigament2d m_ligament;
  private Transform2d m_transform;

  public Link2d(Viz2d viz, String name, double length, double lineWeight, Color color) {
    this(viz, name, length, lineWeight, color, new Transform2d());
  }

  public Link2d(
      Viz2d viz,
      String name,
      double length,
      double lineWeight,
      Color color,
      Transform2d transform) {
    m_root =
        viz.m_viz.getRoot(
            name + " Root",
            viz.metersToPixels(transform.getX()),
            viz.metersToPixels(transform.getY()));
    m_ligament =
        m_root.append(
            new LoggedMechanismLigament2d(
                name + " Ligament",
                viz.metersToPixels(length),
                transform.getRotation().getDegrees(),
                lineWeight,
                new Color8Bit(color)));
    m_transform = transform;
  }

  /** Sets the transform of this link relative to its parent. */
  public void setRelativeTransform(Transform2d transform) {
    m_transform = transform;
  }

  /** Gets the transform of this link relative to its parent. */
  public Transform2d getRelativeTransform() {
    return m_transform;
  }

  /** Adds a child link. */
  public Link2d addLink(Link2d link) {
    m_children.add(link);
    return link;
  }

  protected ArrayList<Link2d> getChildren() {
    return m_children;
  }

  protected void draw(Viz2d viz, double pixelsPerMeter, Transform2d parentTransform) {
    Transform2d transform = parentTransform.plus(m_transform);
    m_root.setPosition(viz.metersToPixels(transform.getX()), viz.metersToPixels(transform.getY()));
    m_ligament.setAngle(transform.getRotation());
  }
}
