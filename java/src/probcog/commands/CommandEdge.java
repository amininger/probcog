package probcog.commands;

import java.io.*;
import java.util.*;

/** Encodes all control laws and associated termination conditions that can
 *  transition a robot from one location to another.
 **/
public class CommandEdge implements Serializable
{
    public double probability = 1.0;

    public String law;
    public HashMap<String, TypedValue> lawParams = new HashMap<String, TypedValue>();

    public String term;
    public HashMap<String, TypedValue> termParams = new HashMap<String, TypedValue>();

    public void addLawParam(String name, TypedValue value)
    {
        lawParams.put(name, value);
    }

    public void addTermParam(String name, TypedValue value)
    {
        termParams.put(name, value);
    }

    public String toString()
    {
        String ls = law+"\n";
        String ts = term+"\n";
        for (String key: lawParams.keySet()) {
            ls += "\t["+key+" = "+lawParams.get(key)+"]\n";
        }
        for (String key: termParams.keySet()) {
            ts += "\t["+key+" = "+termParams.get(key)+"]\n";
        }
        return ls + ts;
    }

    public CommandEdge()
    {
    }

    public CommandEdge(String law, String term)
    {
        this.law = law;
        this.term = term;
    }
}
