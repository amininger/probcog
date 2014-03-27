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

    HashMap<WComponent, ControlLaw> commandMap;

    ControlLawFactory factor

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

        synchronized(thing) {
            // Add some dummy things?
        }
    }
}
