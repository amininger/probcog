package probcog.gui;

import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.vis.*;
import april.util.*;

import probcog.commands.*;
import probcog.util.*;

/** Usage: Double click to place a node. Single click an existing node to
 *  select it, and single click another node to modify edges from the first
 *  to the second. Click and drag to move nodes.
 **/
public class GraphVisEventHandler extends VisEventAdapter
{
    // Parameters
    static final double MAX_SELECT_DISTANCE = 1.0;  // [m]
    static final double DOUBLE_CLICK = 0.3;         // [s]

    // Rendering
    VisWorld vw;

    // The graph we interact with and build in this mode
    MultiGraph<CommandNode, CommandEdge> graph;

    // Whether or not this mode is currently active
    boolean on = false;

    // Rendering state
    int nodeARGB = 0x77a00000;
    int edgeARGB = 0x77a0a000;
    int nodeSelectARGB = 0x7700a000;
    int nodeDraggedARGB = 0x77ffaa00;
    Color nodeHighlight = new Color(nodeARGB, true);
    Color edgeHighlight = new Color(edgeARGB, true);
    Color nodeSelect = new Color(nodeSelectARGB);
    Color nodeDrag = new Color(nodeDraggedARGB);

    // Selection state
    Tic lastClick = new Tic();
    int node0 = -1;
    int node1 = -1;
    int dragNode = -1;

    public GraphVisEventHandler(VisWorld vw, MultiGraph<CommandNode, CommandEdge> graph)
    {
        this.vw = vw;
        this.graph = graph;
        this.on = false;
    }

    public void toggle()
    {
        on = !on;
        if (on) {
            node0 = -1;
            node1 = -1;
            dragNode = -1;
        }
    }

    public boolean isOn()
    {
        return on;
    }

    private int closestNode(double[] xy, double maxRange)
    {
        double bestDist = maxRange;
        int bestNode = -1;
        for (Integer key: graph.getNodes()) {
            CommandNode n = graph.getValue(key);
            double dist = LinAlg.distance(n.getXY(), xy, 2);
            if (dist < bestDist) {
                bestNode = key;
                bestDist = dist;
            }
        }

        return bestNode;
    }

    /** Handlers with lower dispatch order are called first **/
    public int getDispatchOrder()
    {
        return -20; // Must be lower than handler for Simulator
    }

    /** Return true if you've consumed the event. **/
    public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        if (!on) {
            return false;
        }

        // Selection for dragging
        double[] xy = ray.intersectPlaneXY();
        int bestNode = closestNode(xy, MAX_SELECT_DISTANCE);
        if (bestNode >= 0) {
            dragNode = bestNode;
            return true;
        }

        return false;
    }

    public boolean mouseDragged(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        if (!on) {
            return false;
        }

        // Drag around a selected node
        if (dragNode < 0)
            return false;

        double[] xy = ray.intersectPlaneXY();
        synchronized (graph) {
            graph.getValue(dragNode).setXY(xy);
        }

        return true;
    }

    public boolean mouseReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        if (!on) {
            return false;
        }

        // Rendering tidy up
        if (dragNode == node0)
            vw.getBuffer("graph-click").swap();
        dragNode = -1;

        return false;
    }

    public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        VisWorld.Buffer vb = vw.getBuffer("graph-click");
        if (!on) {
            vb.swap();
            return false;
        }

        double[] xy = ray.intersectPlaneXY();
        int bestNode = closestNode(xy, MAX_SELECT_DISTANCE);

        // First click can select an existing point. A second click in a rapid
        // enough time period will create a new node when applicable
        if (lastClick.toctic() > DOUBLE_CLICK) {
            // Case 1: Clicked on an existing point
            //  Case A: No points selected. Select this one
            //  Case B: Point selected. If this is a new point, spawn the edge constructor
            // Case 2: Clicked somewhere new. Reset clicks
            if (bestNode >= 0) {
                if (node0 < 0) {
                    node0 = bestNode;
                } else if (node1 < 0 && bestNode != node0) {
                    node1 = bestNode;
                    new EdgeConstructor(graph, node0, node1);
                    node0 = node1 = -1; // Clean up for next go
                }
            } else {
                node0 = node1 = -1;
            }
        } else {
            // Double clicked! If we weren't clicking on any existing nodes, create one
            if (bestNode < 0) {
                synchronized (graph) {
                    graph.addNode(new CommandNode(xy));
                }
            }
        }

        // Rendering of selections
        CommandNode n0 = graph.getValue(node0);
        if (n0 != null) {
            vb.addBack(new VzPoints(new VisVertexData(n0.getXY()),
                                    new VzPoints.Style(nodeSelect, 7)));
        }
        vb.swap();

        return false;
    }


    public boolean mouseMoved(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
    {
        VisWorld.Buffer vb = vw.getBuffer("graph-move");
        if (!on) {
            vb.swap();
            return false;
        }

        double[] xy = ray.intersectPlaneXY();

        // Find closest point in graph. If close enough, highlight it instead of
        // our currently hovered point
        int bestKey = closestNode(xy, MAX_SELECT_DISTANCE);

        // Draw preview edge if applicable
        if (node0 >= 0 && bestKey >= 0) {
            CommandNode n0 = graph.getValue(node0);
            CommandNode n1 = graph.getValue(bestKey);
            if (n0 != null && n1 != null) {
                vb.addBack(new VzLines(new VisVertexData(n0.getXY(), n1.getXY()),
                                       VzLines.LINES,
                                       new VzLines.Style(edgeHighlight, 3)));
            }
        }

        double[] bestXY = xy;
        if (bestKey >= 0)
            bestXY = graph.getValue(bestKey).getXY();
        vb.addBack(new VzPoints(new VisVertexData(bestXY),
                                new VzPoints.Style(nodeHighlight, 7))); // XXX
        vb.swap();

        return true;
    }

    public boolean mouseWheel(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseWheelEvent e)
    {
        return false;
    }

    public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
    {
        return false;
    }

    public boolean keyTyped(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
    {
        return false;
    }

    public boolean keyReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
    {
        return false;
    }
}
