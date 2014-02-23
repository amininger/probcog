package probcog.robot.util;

import java.io.*;
import java.util.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import javax.swing.event.*;
import java.awt.geom.*;

/** A JComponent which implements basic value versus time plots. When
 * showing multiple channels, all share the same X and Y scaling
 * settings. **/
public class JWaveform extends JPanel
{
    static class Channel
    {
        String name;
        String units;

        Color  color = Color.yellow;
        float    width = 2;

        LinkedList<double[]> xys = new LinkedList<double[]>();
    }

    static class Buddy
    {
        JWaveform jw;
        boolean x, y;
    }

    ArrayList<Channel> channels = new ArrayList<Channel>();
    HashMap<String, Channel> channelMap = new HashMap<String, Channel>();

    int selectedChannelIdx; // which channel do we follow?

    int yGridSize = 100;
    int xGridSize = 100;

    // all channels share same horizontal scale
    int xUnitNotches = 0;

    public static final Color[] colors = new Color[] { Color.yellow, Color.cyan, Color.red, Color.green };

    int textHeight = 15; // height of each line of text
    int channelLabelHeight = 50; // height of the line(s) of text that are painted for each channel's label
    int channelLabelWidth = 200; // width of the label box.

    double ymin, ymax;

    // how much along each axis should we show?
    double xsizes[];
    int xsizeidx;

    double ysizes[];
    int ysizeidx;

    // how many divisions?
    int xdivs;
    int ydivs;

    // which other jwaveforms rescale in sync with us?
    ArrayList<Buddy> buddies = new ArrayList<Buddy>();

    public JWaveform(double xsizes[], int xsizeidx, int xdivs, double ysizes[], int ysizeidx, int ydivs)
    {
        this.xsizes = xsizes;
        this.xsizeidx = xsizeidx;
        this.xdivs = xdivs;
        this.ysizes = ysizes;
        this.ysizeidx = ysizeidx;
        this.ydivs = ydivs;

        addMouseWheelListener(new MyMouseHandler());
        addMouseListener(new MyMouseHandler());
        addKeyListener(new MyKeyAdapter());
/*
        MouseHandler mh = new MouseHandler();
        addMouseMotionListener(mh);
        addMouseListener(mh);
*/
    }

    /** Add a new data point. xy[0] must be the largest value thus far. **/
    public synchronized void addData(String name, double xy[])
    {
        addData(name, xy, 0);
    }

    Channel getChannel(String name)
    {
        Channel c = channelMap.get(name);

        if (c == null) {
            c = new Channel();
            c.name = name;
            c.color = colors[channels.size() % colors.length];
            channels.add(c);
            channelMap.put(name, c);
        }

        return c;
    }

    public synchronized void setWidth(String name, float w)
    {
        Channel c = getChannel(name);
        c.width = w;
    }

    public synchronized void setColor(String name, Color color)
    {
        Channel c = getChannel(name);
        c.color = color;
    }

    /** Add a new data point. xy[0] must be the largest value thus far. **/
    public synchronized void addData(String name, double xy[], double mindx)
    {
        Channel c = getChannel(name);

        if (c.xys.size() > 0) {
            double lastxy[] = c.xys.getLast();
            if (Math.abs(lastxy[0] - xy[0]) < mindx)
                return;
        }

        c.xys.add(new double[] { xy[0], xy[1] });

        repaint();
    }

    public synchronized void paint(Graphics _g)
    {
        Graphics2D g = (Graphics2D) _g;
        int width = getWidth();
        int height = getHeight();

        g.setColor(Color.black);
        g.fillRect(0, 0, width, height);

        if (channels.size() == 0)
            return;

        g.setFont(new Font("Monospaced", Font.PLAIN, 12));

        int plotWidth = width;
        int plotHeight = height;

        double xsize = xsizes[xsizeidx];
        double ysize = ysizes[ysizeidx];

        double xmax = -Double.MAX_VALUE;

        Channel selectedChannel = channels.get(selectedChannelIdx);

        if (selectedChannel.xys.size() != 0) {

            double xy[] = selectedChannel.xys.getLast();

            xmax = Math.max(xmax, xy[0]);

            // when starting up, center around the y value.
            if (ymin == 0 && ymax == 0) {
                ymin = xy[1];
                ymax = xy[1];
            }
        }

        double xmin = xmax - xsize;

        // adjust Y view area so that:
        //    A) the most recent data is visible (is between ymin and ymax)
        //    B) ymax-ymin = ysize
        //    C) the viewport changes by as little as possible subject to the above.

        // enlarge if necessary
        if (ymax - ymin < ysize) {
            double d = ysize - (ymax - ymin);
            ymax += d/2;
            ymin -= d/2;
        }

        // shrink if necessary
        if (ymax - ymin > ysize) {
            double d = (ymax - ymin) - ysize;
            ymax -= d/2;
            ymin += d/2;
        }

        // ensure visibility
        if (true) {
            Channel c = channels.get(selectedChannelIdx);

            if (c.xys.size() > 0) {

                double xy[] = c.xys.getLast();

                double margin = 0.1;
                if (xy[1] + ysize*margin > ymax) {
                    ymax = xy[1] + ysize*margin;
                    ymin = ymax - ysize;
                }

                if (xy[1] - ysize*margin < ymin) {
                    ymin = xy[1] - ysize*margin;
                    ymax = ymin + ysize;
                }
            }
        }

        // prune data
        double maxxsize = 0;
        for (double sz : xsizes)
            maxxsize = Math.max(sz, maxxsize);

        for (Channel c : channels) {
            while (c.xys.size() > 0) {
                double xy[] = c.xys.getFirst();
                if (xy[0] < xmax - maxxsize)
                    c.xys.removeFirst();
                else
                    break;
            }
        }

        // draw y divs
        g.setColor(Color.darkGray);

        for (double y = ymin; y <= ymax; y += ysize / ydivs) {
            double dy = ysize / ydivs;

            int iy = (int) (y / dy);
            double roundy = iy * dy;
            int py = plotHeight - (int) (plotHeight * (roundy - ymin) / (ymax - ymin));
            g.drawLine(0, py, plotWidth, py);

            g.drawString(""+roundy, 0, py);
        }

        // draw x divs
        for (double x = xmin; x <= xmax; x += xsize / xdivs) {
            double dx = xsize / xdivs;

            int ix = (int) (x / dx);
            double roundx = ix * dx;

            int px = (int) (plotWidth * (roundx - xmin) / (xmax - xmin));
            g.drawLine(px, 0, px, plotHeight);
        }

        // draw the channels
        int labely = textHeight;

        for (int cidx = 0; cidx < channels.size(); cidx++) {
            Channel c = channels.get(cidx);

            if (c.xys.size() == 0)
                continue;

            int lastx = Integer.MAX_VALUE, lasty = -1;

            g.setColor(c.color);
//            g.setStroke(new BasicStroke(c.width));

            for (double xy[] : c.xys) {

                int x = (int) (plotWidth * (xy[0] - xmin) / (xmax - xmin));
                int y = plotHeight - (int) (plotHeight * (xy[1] - ymin) / (ymax - ymin));

                if (lastx == x && lasty == y)
                    continue;

                if (lastx != Integer.MAX_VALUE && x >= 0)
                    g.drawLine(lastx, lasty, x, y);

                lastx = x;
                lasty = y;
            }

            String label = String.format("%s %-16f %s",  cidx == selectedChannelIdx ? "> " : "  ", c.xys.getLast()[1], c.name);
            g.drawString(label, 100, labely);
            labely += textHeight;
        }

        g.setColor(Color.darkGray);
        String label = String.format("[YDIV %.3f, XDIV %.3f]", ysize / ydivs, xsize / xdivs);
        g.drawString(label, 100, plotHeight - 3);
        labely += textHeight;
    }

    /** Make the other waveform adjust its view in sync with this one. **/
    public void addViewBuddy(JWaveform jw, boolean x, boolean y)
    {
        Buddy b = new Buddy();
        b.jw = jw;
        b.x = x;
        b.y = y;

        if (jw != this)
            buddies.add(b);
    }

    class MyKeyAdapter extends KeyAdapter
    {
        public void keyPressed(KeyEvent e)
        {
            System.out.println(e.getKeyChar());

            if (e.getKeyChar() == '\t') {
                selectedChannelIdx = (selectedChannelIdx + 1) % channels.size();
            }
        }
    }

    class MyMouseHandler extends MouseAdapter implements MouseWheelListener
    {
        public void mouseWheel(MouseWheelEvent e)
        {
            int mods = e.getModifiersEx();
            boolean shift=(mods&MouseEvent.SHIFT_DOWN_MASK)>0;
            boolean control=(mods&MouseEvent.CTRL_DOWN_MASK)>0;

            int amt = e.getWheelRotation();

            if (shift || control) {
                xsizeidx += amt;
                xsizeidx = Math.max(0, xsizeidx);
                xsizeidx = Math.min(xsizes.length-1, xsizeidx);
            } else {

                ysizeidx += amt;
                ysizeidx = Math.max(0, ysizeidx);
                ysizeidx = Math.min(ysizes.length-1, ysizeidx);
            }

            for (Buddy b : buddies) {
                if (b.x) {
                    b.jw.xsizes = xsizes;
                    b.jw.xsizeidx = xsizeidx;
                }

                if (b.y) {
                    b.jw.ysizes = ysizes;
                    b.jw.ysizeidx = ysizeidx;
                }

                b.jw.repaint();
            }

            repaint();
        }

        public void mouseClicked(MouseEvent e)
        {
            if (e.getButton()==1) {
                selectedChannelIdx = (selectedChannelIdx + 1) % channels.size();
            }
        }
    }
}
