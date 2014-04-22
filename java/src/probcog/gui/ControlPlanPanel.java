package probcog.gui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;

import april.vis.*;

import probcog.commands.*;
import probcog.commands.controls.*;

public class ControlPlanPanel extends JPanel
{
    WAdapter wadapter;
    WDragPanel planPanel = new WDragPanel();

    // XXX Could map to strings quite easily...control laws are harder
    HashMap<WComponent, ControlLaw> commandMap;

    public ControlPlanPanel()
    {
        wadapter = new WAdapter(planPanel);

        setLayout(new BorderLayout());
        add(new JScrollPane(wadapter), BorderLayout.CENTER);
        add(Box.createHorizontalStrut(200));

        rebuild();
    }

    void rebuild()
    {
        planPanel.clear();  // XXX

        Color commandBorder = new Color(220, 220, 220);
        Color commandBackground = new Color(240, 240, 240);

        // Subcomponents?

        planPanel.backgroundColor = Color.white;
        planPanel.grabColor = commandBorder;

        commandMap = new HashMap<WComponent, ControlLaw>();

        // XXX Thought dump: It could be very easy to get caught up in wanting
        // to design this interface and jumping through lots of hoops to make our
        // back end fit the front end, here. At the end of the day, the interface
        // is not the goal. It's a tool to test control law plans, and our hope
        // was to be able to create and modify plans and visualize them easily.
        // If this is not the interface for doing that, then so be it.
        //
        // As a recap, though: our goal is to be able to drag in control laws
        // that we know about from a little box somewhere and execute them.
        // Control laws themselves are pretty easy, in principal. The hard
        // part of this is the termination conditions. Do X until Y. How do
        // we know which termination conditions are valid? Should we care? Could
        // we just apply any termination condition to any control law and call
        // it a day?
        //
        // Challenges with this interface include (but are not limited to) having
        // the ability to define the arbitrary parameters you might want to
        // associate with a control law and making this generalizable if possible.
        //
        // ...
        synchronized(ControlLawFactory.getLock()) {
            // What are we hoping to do here?
        }
    }
}
