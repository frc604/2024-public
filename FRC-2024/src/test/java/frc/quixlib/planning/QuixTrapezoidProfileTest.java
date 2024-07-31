// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.quixlib.planning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.junit.jupiter.api.Test;

class QuixTrapezoidProfileTest {
  private static final double kDt = 0.01;

  /**
   * Asserts "val1" is less than or equal to "val2".
   *
   * @param val1 First operand in comparison.
   * @param val2 Second operand in comparison.
   */
  private static void assertLessThanOrEquals(double val1, double val2) {
    assertTrue(val1 <= val2, val1 + " is greater than " + val2);
  }

  /**
   * Asserts "val1" is within "eps" of "val2".
   *
   * @param val1 First operand in comparison.
   * @param val2 Second operand in comparison.
   * @param eps Tolerance for whether values are near to each other.
   */
  private static void assertNear(double val1, double val2, double eps) {
    assertTrue(
        Math.abs(val1 - val2) <= eps,
        "Difference between " + val1 + " and " + val2 + " is greater than " + eps);
  }

  /**
   * Asserts "val1" is less than or within "eps" of "val2".
   *
   * @param val1 First operand in comparison.
   * @param val2 Second operand in comparison.
   * @param eps Tolerance for whether values are near to each other.
   */
  private static void assertLessThanOrNear(double val1, double val2, double eps) {
    if (val1 <= val2) {
      assertLessThanOrEquals(val1, val2);
    } else {
      assertNear(val1, val2, eps);
    }
  }

  @Test
  void reachesGoal() {
    Constraints constraints = new Constraints(1.75, 0.75);
    State goal = new State(3, 0);
    State state = new State();

    for (int i = 0; i < 450; ++i) {
      QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
    }
    assertEquals(state, goal);
  }

  // Tests that decreasing the maximum velocity in the middle when it is already
  // moving faster than the new max is handled correctly
  @Test
  void posContinousUnderVelChange() {
    Constraints constraints = new Constraints(1.75, 0.75);
    State goal = new State(12, 0);

    QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, new State());
    State state = profile.calculate(kDt);

    double lastPos = state.position;
    for (int i = 0; i < 1600; ++i) {
      if (i == 400) {
        constraints = new Constraints(0.75, 0.75);
      }

      profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
      double estimatedVel = (state.position - lastPos) / kDt;

      if (i >= 400) {
        // Since estimatedVel can have floating point rounding errors, we check
        // whether value is less than or within an error delta of the new
        // constraint.
        assertLessThanOrNear(estimatedVel, constraints.maxVelocity, 1e-4);

        assertLessThanOrEquals(state.velocity, constraints.maxVelocity);
      }

      lastPos = state.position;
    }
    assertEquals(state, goal);
  }

  // There is some somewhat tricky code for dealing with going backwards
  @Test
  void backwards() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(-2, 0);
    State state = new State();

    for (int i = 0; i < 400; ++i) {
      QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
    }
    assertEquals(state, goal);
  }

  @Test
  void switchGoalInMiddle() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(-2, 0);
    State state = new State();

    for (int i = 0; i < 200; ++i) {
      QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
    }
    assertNotEquals(state, goal);

    goal = new State(0.0, 0.0);
    for (int i = 0; i < 550; ++i) {
      QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
    }
    assertEquals(state, goal);
  }

  // Checks to make sure that it hits top speed
  @Test
  void topSpeed() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(4, 0);
    State state = new State();

    for (int i = 0; i < 200; ++i) {
      QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
    }
    assertNear(constraints.maxVelocity, state.velocity, 10e-5);

    for (int i = 0; i < 2000; ++i) {
      QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
    }
    assertEquals(state, goal);
  }

  @Test
  void timingToCurrent() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(2, 0);
    State state = new State();

    for (int i = 0; i < 400; i++) {
      QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
      assertNear(profile.timeLeftUntil(state.position), 0, 2e-2);
    }
  }

  @Test
  void timingToGoal() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(2, 0);

    QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, new State());
    State state = profile.calculate(kDt);

    double predictedTimeLeft = profile.timeLeftUntil(goal.position);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
      if (!reachedGoal && state.equals(goal)) {
        // Expected value using for loop index is just an approximation since
        // the time left in the profile doesn't increase linearly at the
        // endpoints
        assertNear(predictedTimeLeft, i / 100.0, 0.25);
        reachedGoal = true;
      }
    }
  }

  @Test
  void timingBeforeGoal() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(2, 0);

    QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, new State());
    State state = profile.calculate(kDt);

    double predictedTimeLeft = profile.timeLeftUntil(1);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
      if (!reachedGoal && Math.abs(state.velocity - 1) < 10e-5) {
        assertNear(predictedTimeLeft, i / 100.0, 2e-2);
        reachedGoal = true;
      }
    }
  }

  @Test
  void timingToNegativeGoal() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(-2, 0);

    QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, new State());
    State state = profile.calculate(kDt);

    double predictedTimeLeft = profile.timeLeftUntil(goal.position);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
      if (!reachedGoal && state.equals(goal)) {
        // Expected value using for loop index is just an approximation since
        // the time left in the profile doesn't increase linearly at the
        // endpoints
        assertNear(predictedTimeLeft, i / 100.0, 0.25);
        reachedGoal = true;
      }
    }
  }

  @Test
  void timingBeforeNegativeGoal() {
    Constraints constraints = new Constraints(0.75, 0.75);
    State goal = new State(-2, 0);

    QuixTrapezoidProfile profile = new QuixTrapezoidProfile(constraints, goal, new State());
    State state = profile.calculate(kDt);

    double predictedTimeLeft = profile.timeLeftUntil(-1);
    boolean reachedGoal = false;
    for (int i = 0; i < 400; i++) {
      profile = new QuixTrapezoidProfile(constraints, goal, state);
      state = profile.calculate(kDt);
      if (!reachedGoal && Math.abs(state.velocity + 1) < 10e-5) {
        assertNear(predictedTimeLeft, i / 100.0, 2e-2);
        reachedGoal = true;
      }
    }
  }
}
