package probcog.robot.radio;

import java.util.*;

public interface Radio
{
    abstract public boolean sendPacket(byte data[], int offset, int datalen);

    abstract public int getMaxMessageSize();

    abstract public void addListener(RadioListener listener);
}
