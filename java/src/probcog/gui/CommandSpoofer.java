package probcog.gui;

import java.awt.*;
import java.awt.event.*;
import java.io.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.text.*;
import java.util.*;

import lcm.lcm.*;

import april.util.*;
import april.lcmtypes.*;

import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.lcmtypes.*;
import probcog.util.*;

public class CommandSpoofer extends JFrame
{
    private LCM lcm = LCM.getSingleton();
    static final String NAME_PARAM = "__NAME__";

    Map<String, TypedValue> lawMap = new HashMap<String, TypedValue>();
    Map<String, TypedValue> termMap = new HashMap<String, TypedValue>();

    private class ButtonListener implements ActionListener
    {
        public void actionPerformed(ActionEvent e)
        {
            String name = e.getActionCommand();
            if (name.contains("Issue")) {
                issueCommand();
            } else if (name.contains("Stop")) {
                publishStop();
            }
        }
    }

    private class ComboBoxListener implements ItemListener
    {
        TypedParameter tp;
        Map<String, TypedValue> values;

        public ComboBoxListener(TypedParameter tp, Map<String, TypedValue> values)
        {
            this.tp = tp;
            this.values = values;
        }

        public void itemStateChanged(ItemEvent e)
        {
            if (e.getStateChange() != ItemEvent.SELECTED)
                return;
            TypedValue tv = (TypedValue)(e.getItem());
            values.put(tp.getName(), tv);
        }
    }

    private class TextFieldListener implements DocumentListener
    {
        TypedParameter tp;
        Map<String, TypedValue> values;

        public TextFieldListener(TypedParameter tp, Map<String, TypedValue> values)
        {
            this.tp = tp;
            this.values = values;
        }

        public void changedUpdate(DocumentEvent e)
        {
            parse(e);
        }

        public void removeUpdate(DocumentEvent e)
        {
            parse(e);
        }

        public void insertUpdate(DocumentEvent e)
        {
            parse(e);
        }

        private void parse(DocumentEvent e)
        {
            Document doc = e.getDocument();
            try {
                String text = doc.getText(0, doc.getLength());
                if (text.length() < 1)
                    values.put(tp.getName(), null);
                TypedValue tv;
                try {
                    switch (tp.getType()) {
                        case TypedValue.TYPE_BYTE:
                            byte b = Byte.parseByte(text);
                            tv = new TypedValue(b);
                            break;
                        case TypedValue.TYPE_SHORT:
                            short s = Short.parseShort(text);
                            tv = new TypedValue(s);
                            break;
                        case TypedValue.TYPE_INT:
                            int i = Integer.parseInt(text);
                            tv = new TypedValue(i);
                            break;
                        case TypedValue.TYPE_LONG:
                            long l = Long.parseLong(text);
                            tv = new TypedValue(l);
                            break;
                        case TypedValue.TYPE_FLOAT:
                            float f = Float.parseFloat(text);
                            tv = new TypedValue(f);
                            break;
                        case TypedValue.TYPE_DOUBLE:
                            double d = Double.parseDouble(text);
                            tv = new TypedValue(d);
                            break;
                        default:
                            tv = new TypedValue(text);
                    }
                } catch (NumberFormatException ex) { return; }
                values.put(tp.getName(), tv);
            } catch (BadLocationException ex) {
                assert(false);
            }
        }
    }

    private class CheckBoxListener implements ItemListener
    {
        JComponent component;
        TypedParameter tp;
        Map<String, TypedValue> values;

        public CheckBoxListener(JComponent component, TypedParameter tp, Map<String, TypedValue> values)
        {
            this.component = component;
            this.tp = tp;
            this.values = values;
        }

        public void itemStateChanged(ItemEvent e)
        {
            JCheckBox jcb = (JCheckBox)(e.getItem());
            component.setEnabled(jcb.isSelected());
            if (jcb.isSelected()) {
                values.put(tp.getName(), null);
            } else {
                values.remove(tp.getName());
            }
        }
    }

    private class NameBoxListener implements ItemListener
    {
        JPanel pane;
        Map<String, Collection<TypedParameter> > params;
        Map<String, TypedValue> values;

        public NameBoxListener(JPanel pane, String name, Map<String, TypedValue> values, Map<String, Collection<TypedParameter> > params)
        {
            this.pane = pane;
            this.params = params;
            this.values = values;

            // Pane layout
            this.pane.setLayout(new BoxLayout(this.pane, BoxLayout.Y_AXIS));
            rebuild(name);
        }

        public void itemStateChanged(ItemEvent e)
        {
            if (e.getStateChange() != ItemEvent.SELECTED)
                return;
            String name = (String)(e.getItem());
            rebuild(name);
        }

        private void rebuild(String name)
        {
            Collection<TypedParameter> p = params.get(name);

            // Reset information
            pane.removeAll();
            values.clear();

            // Add special __NAME__ parameter for edge construction
            values.put(NAME_PARAM, new TypedValue(name));

            // Update panel contents and construct appropriate listeners
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
                internalPane.setBorder(BorderFactory.createTitledBorder(title));

                // Deal with non-required parameters
                JCheckBox jcb = new JCheckBox();
                jcb.setSelected(tp.isRequired());
                jcb.setEnabled(!tp.isRequired());
                if (tp.isRequired())
                    values.put(tp.getName(), null);
                internalPane.add(jcb);

                if (tp.hasValid()) {
                    // Can display options in a combo box
                    TypedValue[] choices = tp.getValid().toArray(new TypedValue[0]);
                    JComboBox combo = new JComboBox(choices);
                    combo.addItemListener(new ComboBoxListener(tp, values));
                    values.put(tp.getName(), choices[0]);

                    combo.setEnabled(tp.isRequired());
                    jcb.addItemListener(new CheckBoxListener(combo, tp, values));
                    internalPane.add(combo);
                } else {
                    JTextField tf = new JTextField(8);
                    tf.getDocument().addDocumentListener(new TextFieldListener(tp, values));

                    tf.setEnabled(tp.isRequired());
                    jcb.addItemListener(new CheckBoxListener(tf, tp, values));
                    internalPane.add(tf);
                }

                pane.add(internalPane);
            }
            pane.add(Box.createRigidArea(new Dimension(200,0)));

            pack();
        }
    }

    public CommandSpoofer()
    {
        super("Command Spoofer");

        System.out.println("Spawning command spoofer");
        setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));

        // Will need to add in some stuff for composite control laws
        JPanel selectionPanel = new JPanel();
        JPanel lawPanel = new JPanel();
        JPanel termPanel = new JPanel();
        selectionPanel.setLayout(new BoxLayout(selectionPanel, BoxLayout.X_AXIS));
        lawPanel.setLayout(new BoxLayout(lawPanel, BoxLayout.Y_AXIS));
        termPanel.setLayout(new BoxLayout(termPanel, BoxLayout.Y_AXIS));

        lawPanel.setBorder(BorderFactory.createTitledBorder("Control Law"));
        termPanel.setBorder(BorderFactory.createTitledBorder("Termination Condition"));

        selectionPanel.add(lawPanel);
        selectionPanel.add(termPanel);
        add(selectionPanel);

        // Fill panels
        ControlLawFactory clf = ControlLawFactory.getSingleton();
        ConditionTestFactory ctf = ConditionTestFactory.getSingleton();
        fillPanel(lawPanel, lawMap, clf.getParameters());
        fillPanel(termPanel, termMap, ctf.getParameters());

        // Bottom buttons
        Button issue, stop;
        issue = new Button("Issue Command");
        issue.addActionListener(new ButtonListener());
        stop = new Button("Stop");
        stop.addActionListener(new ButtonListener());
        JPanel buttonPanel = new JPanel();
        buttonPanel.setLayout(new BoxLayout(buttonPanel, BoxLayout.X_AXIS));
        buttonPanel.add(issue);
        buttonPanel.add(stop);
        add(buttonPanel);

        pack();
        setVisible(true);
    }

    private void fillPanel(JPanel pane, Map<String, TypedValue> values, Map<String, Collection<TypedParameter> > params)
    {
        JPanel internalPane = new JPanel();
        JComboBox nameBox = new JComboBox(params.keySet().toArray(new String[0]));
        nameBox.addItemListener(new NameBoxListener(internalPane,
                                                     (String)(nameBox.getSelectedItem()),
                                                     values,
                                                     params));
        // XXX What about this am I worried about? There was an issue at some point
        pane.add(nameBox);
        pane.add(internalPane);
    }

    private static int id = 0;
    private String lastControl;
    private void issueCommand()
    {
        ArrayList<String> nullParameters = new ArrayList<String>();
        for (String key: lawMap.keySet()) {
            if (lawMap.get(key) == null)
                nullParameters.add(key);
        }
        for (String key: termMap.keySet()) {
            if (termMap.get(key) == null)
                nullParameters.add(key);
        }

        // Null parameters represent unset values. Deal with these accordingly.
        if (nullParameters.size() > 0) {
            // XXX Would be nice to highlight offending boxes in red...
            System.out.println("WRN: Could not issue command. The following parameters were unset:");
            for (String s: nullParameters)
                System.out.println("\t"+s);
            return;
        }

        // Issue the appropriate command
        control_law_t cl = new control_law_t();
        cl.utime = TimeUtil.utime();
        cl.name = lawMap.get(NAME_PARAM).toString();
        cl.id = id++;
        cl.num_params = lawMap.size()-1;
        cl.param_names = new String[cl.num_params];
        cl.param_values = new typed_value_t[cl.num_params];

        int idx = 0;
        for (String key: lawMap.keySet()) {
            if (key.equals(NAME_PARAM)) {
                continue;
            }
            cl.param_names[idx] = key;
            cl.param_values[idx] = lawMap.get(key).toLCM();
            idx++;
        }

        condition_test_t ct = new condition_test_t();
        ct.name = termMap.get(NAME_PARAM).toString();
        ct.num_params = termMap.size()-1;
        ct.param_names = new String[ct.num_params];
        ct.param_values = new typed_value_t[ct.num_params];
        ct.compare_type = condition_test_t.CMP_GT;
        ct.compared_value = (new TypedValue(0)).toLCM();

        idx = 0;
        for (String key: termMap.keySet()) {
            if (key.equals(NAME_PARAM)) {
                continue;
            }
            ct.param_names[idx] = key;
            ct.param_values[idx] = termMap.get(key).toLCM();
            idx++;
        }
        cl.termination_condition = ct;

        System.out.printf("%s\n", cl.name);
        for (int i = 0; i < cl.num_params; i++) {
            System.out.printf("\t%s: %s\n", cl.param_names[i],  cl.param_values[i].value);
        }

        System.out.printf("%s\n", ct.name);
        for (int i = 0; i < ct.num_params; i++) {
            System.out.printf("\t%s: %s\n", ct.param_names[i],  ct.param_values[i].value);
        }

        lcm.publish("SOAR_COMMAND_TX", cl);
        lastControl = cl.name;
    }

    private void publishStop()
    {
        control_law_status_t status = new control_law_status_t();
        status.id = id - 1;
        status.name = lastControl;
        status.status = "DESTROY";

        control_law_status_list_t status_list = new control_law_status_list_t();
        status_list.utime = TimeUtil.utime();
        status_list.nstatuses = 1;
        status_list.statuses = new control_law_status_t[status_list.nstatuses];
        status_list.statuses[0] = status;

        lcm.publish("CONTROL_LAW_STATUS_TX", status_list);
    }

    // No longer used
    private void closeGUI()
    {
        dispose();
    }

    static public void main(String[] args)
    {
        CommandSpoofer cs = new CommandSpoofer();
    }

}
