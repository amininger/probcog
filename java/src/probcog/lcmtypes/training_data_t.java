/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package probcog.lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class training_data_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public int num_labels;
    public probcog.lcmtypes.training_label_t labels[];
 
    public training_data_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x4526df4db6797345L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(probcog.lcmtypes.training_data_t.class))
            return 0L;
 
        classes.add(probcog.lcmtypes.training_data_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + probcog.lcmtypes.training_label_t._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        outs.writeInt(this.num_labels); 
 
        for (int a = 0; a < this.num_labels; a++) {
            this.labels[a]._encodeRecursive(outs); 
        }
 
    }
 
    public training_data_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public training_data_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static probcog.lcmtypes.training_data_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        probcog.lcmtypes.training_data_t o = new probcog.lcmtypes.training_data_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.num_labels = ins.readInt();
 
        this.labels = new probcog.lcmtypes.training_label_t[(int) num_labels];
        for (int a = 0; a < this.num_labels; a++) {
            this.labels[a] = probcog.lcmtypes.training_label_t._decodeRecursiveFactory(ins);
        }
 
    }
 
    public probcog.lcmtypes.training_data_t copy()
    {
        probcog.lcmtypes.training_data_t outobj = new probcog.lcmtypes.training_data_t();
        outobj.utime = this.utime;
 
        outobj.num_labels = this.num_labels;
 
        outobj.labels = new probcog.lcmtypes.training_label_t[(int) num_labels];
        for (int a = 0; a < this.num_labels; a++) {
            outobj.labels[a] = this.labels[a].copy();
        }
 
        return outobj;
    }
 
}

