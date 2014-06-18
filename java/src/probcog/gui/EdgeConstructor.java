package probcog.gui;

import java.awt.*;
import java.awt.event.*;
import java.io.*;
import javax.swing.*;
import java.util.*;

import april.util.*;

import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.util.*;

public class EdgeConstructor extends JFrame
{
    MultiGraph<CommandNode, CommandEdge> graph;
    int n0, n1;

    private class ComboBoxListener implements ItemListener
    {
        JPanel pane;
        Map<String, Collection<TypedParameter> > params;

        public ComboBoxListener(JPanel pane, Map<String, Collection<TypedParameter> > params)
        {
            this.pane = pane;
            this.params = params;

            // Pane layout
            this.pane.setLayout(new BoxLayout(this.pane, BoxLayout.Y_AXIS));
        }

        public void itemStateChanged(ItemEvent e)
        {
            if (e.getStateChange() != ItemEvent.SELECTED)
                return;
            String name = (String)(e.getItem());
            Collection<TypedParameter> p = params.get(name);

            // Update panel contents
            pane.removeAll();
            for (TypedParameter tp: p) {
                JPanel internalPane = new JPanel();
                internalPane.setLayout(new BoxLayout(internalPane, BoxLayout.X_AXIS));
                String title = tp.getName();
                if (tp.hasRange()) {
                    TypedValue[] range = tp.getRange();
                    switch (tp.getType()) {
                        case TypedValue.TYPE_BYTE:
                            title = String.format("%s (%d, %d)", title, range[0].getByte(), range[1].getByte());
                            break;
                        case TypedValue.TYPE_SHORT:
                            title = String.format("%s (%d, %d)", title, range[0].getShort(), range[1].getShort());
                            break;
                        case TypedValue.TYPE_INT:
                            title = String.format("%s (%d, %d)", title, range[0].getInt(), range[1].getInt());
                            break;
                        case TypedValue.TYPE_LONG:
                            title = String.format("%s (%l, %l)", title, range[0].getLong(), range[1].getLong());
                            break;
                        case TypedValue.TYPE_FLOAT:
                            title = String.format("%s (%.3f, %.3f)", title, range[0].getFloat(), range[1].getFloat());
                            break;
                        case TypedValue.TYPE_DOUBLE:
                            title = String.format("%s (%.3f, %.3f)", title, range[0].getDouble(), range[1].getDouble());
                            break;
                        default:
                            System.err.println("ERR: Range not supported for type.");
                            break;
                    }
                }
                // XXX Truncation issues and general vertical spacing/sizing issues as well
                internalPane.setBorder(BorderFactory.createTitledBorder(title));

                JCheckBox jcb = new JCheckBox();
                jcb.setSelected(tp.isRequired());
                jcb.setEnabled(!tp.isRequired());
                // XXX Action handlers for all of this
                internalPane.add(jcb);

                if (tp.hasValid()) {
                    // Can display options in a combo box
                    TypedValue[] choices = tp.getValid().toArray(new TypedValue[0]);
                    JComboBox combo = new JComboBox(choices);
                    internalPane.add(combo);
                } else {
                    JTextField tf = new JTextField(8);
                    internalPane.add(tf);
                }
                internalPane.add(Box.createHorizontalGlue());
                pane.add(internalPane);
            }

            pack(); // XXX
        }
    }

    public EdgeConstructor(MultiGraph<CommandNode, CommandEdge> graph, int n0, int n1)
    {
        super("Edge Constructor "+n0+"-->"+n1);
        this.graph = graph;
        this.n0 = n0;
        this.n1 = n1;

        System.out.println("Spawning edge constructor");

        // Will need to add in some stuff for composite control laws
        JPanel lawPanel = new JPanel();
        JPanel termPanel = new JPanel();
        lawPanel.setLayout(new BoxLayout(lawPanel, BoxLayout.Y_AXIS));
        termPanel.setLayout(new BoxLayout(termPanel, BoxLayout.Y_AXIS));
        lawPanel.setBorder(BorderFactory.createTitledBorder("Control Law"));
        termPanel.setBorder(BorderFactory.createTitledBorder("Termination Condition"));
        setLayout(new BoxLayout(getContentPane(), BoxLayout.X_AXIS));
        add(lawPanel);
        add(termPanel);

        // Fill panels
        ControlLawFactory clf = ControlLawFactory.getSingleton();
        ConditionTestFactory ctf = ConditionTestFactory.getSingleton();
        fillPanel(lawPanel, clf.getParameters());
        fillPanel(termPanel, ctf.getParameters());

        // Bottom buttons XXX

        pack();
        setVisible(true);
    }

    private void fillPanel(JPanel pane, Map<String, Collection<TypedParameter> > params)
    {
        JPanel internalPane = new JPanel();
        JComboBox nameBox = new JComboBox(params.keySet().toArray(new String[0]));
        nameBox.addItemListener(new ComboBoxListener(internalPane, params));
        pane.add(nameBox);
        pane.add(internalPane);
        pane.add(Box.createVerticalGlue());
    }
}
