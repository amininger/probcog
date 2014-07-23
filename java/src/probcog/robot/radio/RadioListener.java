package probcog.robot.radio;

/**
  * Generic listening interface for radios.
  * Note: packets received may be partial packets
  */
public interface RadioListener
{
    public void packetReceived(Radio radio, int sourceAddr, byte data[]);
}
