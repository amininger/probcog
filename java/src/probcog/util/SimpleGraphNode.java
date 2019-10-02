package soargroup.rosie.mobilesim.util;

import java.io.*;
import java.util.*;

public class SimpleGraphNode
{
    static long ID_COUNTER = 0;
    Long nodeID;

    public SimpleGraphNode()
    {
        this(ID_COUNTER++);
    }

    public SimpleGraphNode(long id)
    {
        nodeID = id;
    }

    public int hashCode()
    {
        return nodeID.hashCode();
    }

    public boolean equals(Object o)
    {
        if (o == null)
            return false;
        if (!(o instanceof SimpleGraphNode))
            return false;
        SimpleGraphNode n = (SimpleGraphNode)o;
        return nodeID.equals(n.nodeID);
    }
}
