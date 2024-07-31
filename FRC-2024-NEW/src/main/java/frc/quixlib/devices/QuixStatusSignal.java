package frc.quixlib.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp;
import java.util.function.Function;

/** A wrapper around Phoenix6's StatusSignal that handles unit conversions. */
public class QuixStatusSignal<T> {
  private final StatusSignal<T> m_statusSignal;
  private final Function<Double, Double> m_fromNativeUnits;

  public QuixStatusSignal(final StatusSignal<T> statusSignal) {
    this(statusSignal, value -> value);
  }

  public QuixStatusSignal(
      final StatusSignal<T> statusSignal, final Function<Double, Double> fromNativeUnits) {
    m_statusSignal = statusSignal.clone();
    m_fromNativeUnits = fromNativeUnits;
  }

  public StatusCode setUpdateFrequency(double frequencyHz, double timeoutSeconds) {
    return m_statusSignal.setUpdateFrequency(frequencyHz, timeoutSeconds);
  }

  public double getAppliedUpdateFrequency() {
    return m_statusSignal.getAppliedUpdateFrequency();
  }

  public void refresh() {
    m_statusSignal.refresh();
  }

  public void waitForUpdate(final double timeoutSec) {
    m_statusSignal.waitForUpdate(timeoutSec);
  }

  public T getRawValue() {
    return m_statusSignal.getValue();
  }

  public double getUnitConvertedValue() {
    return m_fromNativeUnits.apply(m_statusSignal.getValueAsDouble());
  }

  public Timestamp getTimestamp() {
    return m_statusSignal.getTimestamp();
  }

  public static BaseStatusSignal[] toBaseStatusSignals(final QuixStatusSignal<?>... quixSignals) {
    final BaseStatusSignal[] signals = new BaseStatusSignal[quixSignals.length];
    for (int i = 0; i < quixSignals.length; i++) {
      signals[i] = quixSignals[i].m_statusSignal;
    }
    return signals;
  }

  public static double getLatencyCompensatedValue(
      final QuixStatusSignal<?> signal, final QuixStatusSignal<?> signalSlope) {
    return signal.getUnitConvertedValue()
        + (signalSlope.getUnitConvertedValue() * signal.getTimestamp().getLatency());
  }
}
