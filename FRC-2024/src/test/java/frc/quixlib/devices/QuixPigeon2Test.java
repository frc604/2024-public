package frc.quixlib.devices;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.*;

public class QuixPigeon2Test {
  static final double kTol = 5e-2;
  static final double kCANDelayTime = 0.5; // Extra long because sim can run slow.

  @Test
  public void testYaw() throws InterruptedException {
    QuixPigeon2 pigeon = new QuixPigeon2(new CANDeviceID(1));
    Timer.delay(0.020);

    pigeon.continuousYawSignal().waitForUpdate(kCANDelayTime);
    assertEquals(0.0, pigeon.getContinuousYaw(), kTol);

    // Simulate continuous yaw.
    for (double yaw = 0.0; yaw < 6.0 * Math.PI; yaw += 0.1) {
      pigeon.setSimContinuousYaw(yaw);
    }
    pigeon.setSimContinuousYaw(6.0 * Math.PI);
    Timer.delay(0.020);
    pigeon.continuousYawSignal().waitForUpdate(kCANDelayTime);
    assertEquals(6.0 * Math.PI, pigeon.getContinuousYaw(), kTol);

    // Test zeroing.
    pigeon.zeroContinuousYaw();
    Timer.delay(0.020);
    pigeon.continuousYawSignal().waitForUpdate(kCANDelayTime);
    assertEquals(0.0, pigeon.getContinuousYaw(), kTol);

    // Test zero to offset.
    pigeon.setContinuousYaw(Math.PI);
    Timer.delay(0.020);
    pigeon.continuousYawSignal().waitForUpdate(kCANDelayTime);
    assertEquals(Math.PI, pigeon.getContinuousYaw(), kTol);

    // Simulate continuous yaw.
    for (double yaw = 6.0 * Math.PI; yaw < 12.0 * Math.PI; yaw += 0.1) {
      pigeon.setSimContinuousYaw(yaw);
    }
    pigeon.setSimContinuousYaw(12.0 * Math.PI);
    Timer.delay(0.020);
    pigeon.continuousYawSignal().waitForUpdate(kCANDelayTime);
    assertEquals(7.0 * Math.PI, pigeon.getContinuousYaw(), kTol);
  }
}
