package probcog.servo;

import java.io.*;
import java.util.Arrays;

import orc.*;

import april.dynamixel.*;

/** Implementation of dynamixel communcation bus for servos
 *  connected to a uorc board.
 **/
public class OrcBus extends AbstractBus
{
    Orc orc;

    boolean verbose = false;

    public OrcBus(Orc orc_)
    {
        orc = orc_;
    }

    /** Send an instruction to the specified servo, retrying as necessary */
    public synchronized byte[] sendCommand(int id,
                                           int instruction,
                                           byte[] parameters,
                                           boolean retry)
    {
        do {
            byte[] resp = sendCommandRaw(id, instruction, parameters);

            if (resp == null || resp.length < 1) {
                if (verbose)
                    System.err.printf("ERR: OrcBus id=%d error: short response\n",
                                      id);
                continue;
            }

            if (resp[0] != 0) {
                int code = (resp[0]) & 0xff;

                int errormask = AbstractServo.ERROR_ANGLE_LIMIT |
                                AbstractServo.ERROR_VOLTAGE |
                                AbstractServo.ERROR_OVERLOAD;

                if ((code & (~errormask)) != 0)
                    continue;
            }

            return resp;
        } while (retry && true);//retryEnable);

        return null;
    }

    /** Send an instruction with the specified parameters. The error code,
     *  body, and checksum of the response are returned (the initial 4 byes of
     *  header are removed)
     **/
    protected byte[] sendCommandRaw(int id,
                                    int instruction,
                                    byte[] parameters)
    {
        int parameterlen = (parameters == null) ? 0 : parameters.length;
        byte[] cmd = new byte[6 + parameterlen];
        cmd[0] = (byte) 255; // magic
        cmd[1] = (byte) 255; // magic
        cmd[2] = (byte) id;  // servo id
        cmd[3] = (byte) (parameterlen + 2); // length
        cmd[4] = (byte) instruction;

        if (parameters != null) {
            for (int i = 0; i < parameters.length; i++)
                cmd[5 + i] = parameters[i];
        }

        if (true) {
            // Compute checksum
            int checksum = 0;
            for (int i = 2; i < cmd.length - 1; i++) {
                checksum += (cmd[i] & 0xff);
            }
            cmd[5 + parameterlen] = (byte) (checksum ^ 0xff);
        }

        OrcResponse resp = orc.doCommand(0x01007a00, cmd);
        if (verbose) {
            resp.print();
            dump(cmd);
        }

        // Read relevent bits
        try {
            int len = resp.ins.read();
            int ff = resp.ins.read();
            ff = resp.ins.read();
            int sid = resp.ins.read();
            if (sid != id) {
                return null;    // XXX
            }
            int paramlen = resp.ins.read();
            byte buf[] = new byte[paramlen];
            for (int i = 0; i < paramlen; i++) {
                buf[i] = (byte)(resp.ins.read() & 0xff);
            }
            return buf;
        } catch (IOException ioex) {
            return null;    // XXX
        }
    }

    static void dump(byte[] buf)
    {
        for (int i = 0; i < buf.length; i++) {
            System.out.printf("%02x ", buf[i] & 0xff);
        }
        System.out.printf("\n");
    }
}
