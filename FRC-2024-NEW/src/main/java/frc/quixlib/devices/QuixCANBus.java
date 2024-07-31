package frc.quixlib.devices;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class QuixCANBus {
  private final String m_canbusName;
  private final String m_loggingName;
  private final CANBus m_canBus;

  private final CANBusInputsAutoLogged m_inputs = new CANBusInputsAutoLogged();

  @AutoLog
  public static class CANBusInputs {
    protected StatusCode status = StatusCode.OK;
    protected double busUtilization = 0.0;
    protected int busOffCount = 0;
    protected int txFullCount = 0;
    protected int REC = 0;
    protected int TEC = 0;
    protected boolean isNetworkFD = false;
  }

  public QuixCANBus() {
    this("rio");
  }

  public QuixCANBus(final String canbusName) {
    m_canbusName = canbusName;
    m_loggingName = "Inputs/CANBus [" + m_canbusName + "]";
    m_canBus = new CANBus(canbusName);
  }

  public void updateInputs() {
    CANBusStatus status = m_canBus.getStatus();
    m_inputs.status = status.Status;
    m_inputs.busUtilization = status.BusUtilization;
    m_inputs.busOffCount = status.BusOffCount;
    m_inputs.txFullCount = status.TxFullCount;
    m_inputs.REC = status.REC;
    m_inputs.TEC = status.TEC;
    m_inputs.isNetworkFD = m_canBus.isNetworkFD();
    Logger.processInputs(m_loggingName, m_inputs);
  }
}
