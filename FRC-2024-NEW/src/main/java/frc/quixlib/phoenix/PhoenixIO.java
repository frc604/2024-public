package frc.quixlib.phoenix;

import com.ctre.phoenix6.StatusCode;

public interface PhoenixIO {
  /** Performs a non-blocking update on the inputs. */
  public StatusCode updateInputs();

  /** Performs a blocking update on the inputs. */
  public StatusCode waitForInputs(final double timeoutSec);
}
