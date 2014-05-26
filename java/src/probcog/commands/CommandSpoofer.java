package probcog.commands;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import javax.swing.*;
import javax.swing.event.*;

import lcm.lcm.*;

import april.util.*;

import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.lcmtypes.*;
// import java.awt.event.ActionEvent;
// import java.awt.event.ActionListener;
// import java.io.IOException;
// import java.util.ArrayList;
// import java.util.HashMap;
// import java.util.Map;

// import javax.swing.JButton;
// import javax.swing.JMenu;

// import lcm.lcm.LCM;
// import lcm.lcm.LCMDataInputStream;
// import lcm.lcm.LCMSubscriber;
// import probcog.arm.ArmStatus;
// import probcog.commands.TypedValue;
// import probcog.lcmtypes.robot_action_t;
// import probcog.lcmtypes.robot_command_t;
// import probcog.lcmtypes.set_state_command_t;
// import probcog.lcmtypes.control_law_t;
// import probcog.lcmtypes.control_law_status_t;
// import probcog.lcmtypes.condition_test_t;
// import probcog.lcmtypes.typed_value_t;
// import sml.Agent;
// import sml.Agent.OutputEventInterface;
// import sml.Agent.RunEventInterface;
// import sml.Identifier;
// import sml.WMElement;
// import sml.smlRunEventId;
// import april.config.Config;
// import april.config.ConfigFile;
// import april.jmat.LinAlg;
// import april.jmat.MathUtil;
// import april.util.TimeUtil;
// import probcog.rosie.world.Pose;
// import probcog.rosie.world.SVSCommands;
// import probcog.rosie.world.WMUtil;
// import probcog.rosie.world.WorldModel;

public class CommandSpoofer extends JFrame
{
    LCM lcm = LCM.getSingleton();

    private JPanel commandParamPanel, testParamPanel;
    private JList commandList, testList;

    final private String COMMAND = "COMMAND";
    final private String TEST = "TEST";

    Map<String, Collection<TypedParameter>> clParams, ctParams;

    private JPanel controlCards, testCards;
    private condition_test_t ct;
    control_law_t cl;

    public void addComponentToPane(Container pane)
    {
        // Get control laws and termination conditions
        ControlLawFactory clfactory = ControlLawFactory.getSingleton();
        ConditionTestFactory ctfactory = ConditionTestFactory.getSingleton();
        clParams = clfactory.getParameters();
        ctParams = ctfactory.getParameters();
        ct = new condition_test_t();
        cl = new control_law_t();

        // Create panel for the control laws
        JPanel controlPane = new JPanel(); //use FlowLayout
        String[] controlLaws = clParams.keySet().toArray(new String[0]);
        JComboBox controlCombo = new JComboBox(controlLaws);
        controlCombo.setEditable(false);
        controlCombo.addItemListener(new ControlChangeListener());
        controlPane.add(controlCombo);

        // Create the panel that contains the control law "cards" and
        // create those cards
        controlCards = new JPanel(new CardLayout());
        for(String law: controlLaws) {
            JPanel card = createPanel(clParams.get(law), COMMAND);
            controlCards.add(card, law);
        }

        // Create panel for the control laws
        JPanel testPane = new JPanel(); //use FlowLayout
        String[] tests = ctParams.keySet().toArray(new String[0]);
        JComboBox testCombo = new JComboBox(tests);
        testCombo.setEditable(false);
        testCombo.addItemListener(new TestChangeListener());
        testPane.add(testCombo);

        // Create the panel that contains the control law "cards" and
        // create those cards
        testCards = new JPanel(new CardLayout());
        for(String test: tests) {
            JPanel card = createPanel(ctParams.get(test), TEST);
            testCards.add(card, test);
        }

        // Create button for sending the lcm message
        JPanel buttonPane = new JPanel();
        JButton sendButton = new JButton("Send Command");
        sendButton.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    publishControlLaw();
                }
            });
        buttonPane.add(sendButton);

        // Add everything to our panel
        pane.add(controlPane);
        pane.add(controlCards);
        pane.add(testPane);
        pane.add(testCards);
        pane.add(buttonPane);
    }

    private JPanel createPanel(Collection<TypedParameter> params, String cardType)
    {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        TypedParameter[] paramsArray = params.toArray(new TypedParameter[0]);

        for(int i=0; i<paramsArray.length; i++)
        {
            TypedParameter tp = paramsArray[i];
            String paramName = tp.getName();
            String type = typeToString(tp.getType());

            JPanel jp = new JPanel();
            jp.add(new JLabel(paramName, JLabel.LEFT));
            final int expectedType = tp.getType();
            final int paramID = i;
            final String commandType = cardType;

            if(tp.hasValid()) {
                TypedValue[] validArray = tp.getValid().toArray(new TypedValue[0]);
                String[] validChoices = new String[validArray.length];
                for(int j=0; j<validChoices.length; j++) {
                    validChoices[j] = validArray[j].toString();
                }

                JComboBox validCombo = new JComboBox(validChoices);
                validCombo.setEditable(false);
                validCombo.addItemListener(new ItemListener() {
                        public void itemStateChanged(ItemEvent event) {
                            if (event.getStateChange() == ItemEvent.SELECTED) {
                                String selection = (String) event.getItem();
                                typed_value_t tv = getTypedValueLCM(expectedType, selection);
                                if(COMMAND.equals(commandType))
                                    cl.param_values[paramID] = tv;
                                else if(TEST.equals(commandType))
                                    ct.param_values[paramID] = tv;
                            }
                        }
                    });


                jp.add(validCombo);
            }
            else {
                if(tp.hasRange()) {
                    TypedValue[] range = tp.getRange();
                    String rangeLabel = "("+range[0].toString()+", "+range[1].toString()+")";
                    jp.add(new JLabel(rangeLabel));
                }
                else {
                    String typeLabel = "("+type+")";
                    jp.add(new JLabel(typeLabel));
                }

                final JTextField tf = new JTextField(8);
                tf.getDocument().addDocumentListener(new DocumentListener() {
                        public void changedUpdate(DocumentEvent e) {
                            updateLCM();
                        }
                        public void removeUpdate(DocumentEvent e) {
                            updateLCM();
                        }
                        public void insertUpdate(DocumentEvent e) {
                            updateLCM();
                        }

                        public void updateLCM() {
                            typed_value_t tv = getTypedValueLCM(expectedType, tf.getText());
                            if(COMMAND.equals(commandType)){
                                cl.param_values[paramID] = tv;
                            }
                            else if(TEST.equals(commandType)){
                                ct.param_values[paramID] = tv;
                            }
                        }
                    });

                jp.add(tf);

            }

            panel.add(jp);
        }


        return panel;
    }

    private typed_value_t getTypedValueLCM(int type, String input)
    {
        switch (type) {
            case TypedValue.TYPE_BOOLEAN:
                return TypedValue.wrap(Boolean.parseBoolean(input));
            case TypedValue.TYPE_BYTE:
                return TypedValue.wrap(Byte.parseByte(input));
            case TypedValue.TYPE_SHORT:
                return TypedValue.wrap(Short.parseShort(input));
            case TypedValue.TYPE_INT:
                return TypedValue.wrap(Integer.parseInt(input));
            case TypedValue.TYPE_LONG:
                return TypedValue.wrap(Long.parseLong(input));
            case TypedValue.TYPE_FLOAT:
                return TypedValue.wrap(Float.parseFloat(input));
            case TypedValue.TYPE_DOUBLE:
                return TypedValue.wrap(Double.parseDouble(input));
            case TypedValue.TYPE_STRING:
                return TypedValue.wrap(input);
        }

        return new typed_value_t();
    }

    private String typeToString(int type)
    {
        switch (type) {
            case TypedValue.TYPE_BOOLEAN:
                return "boolean";
            case TypedValue.TYPE_BYTE:
                return "byte";
            case TypedValue.TYPE_SHORT:
                return "short";
            case TypedValue.TYPE_INT:
                return "int";
            case TypedValue.TYPE_LONG:
                return "long";
            case TypedValue.TYPE_FLOAT:
                return "float";
            case TypedValue.TYPE_DOUBLE:
                return "double";
            case TypedValue.TYPE_STRING:
                return "string";
        }

        return "UNKNOWN";
    }

    private static int id = 0;
    private void publishControlLaw()
    {
        assert(cl != null && ct != null);

        cl.id = id;
        id ++;
        cl.utime = TimeUtil.utime();

        cl.termination_condition = ct;
        lcm.publish("SOAR_COMMAND", cl);
    }

    class ControlChangeListener implements ItemListener
    {
        public void itemStateChanged(ItemEvent event)
        {
            if (event.getStateChange() == ItemEvent.SELECTED) {
                cl.name = (String) event.getItem();
                cl.num_params = clParams.get(cl.name).size();
                cl.param_names = new String[cl.num_params];
                cl.param_values = new typed_value_t[cl.num_params];

                if(cl.num_params > 0) {
                    TypedParameter[] paramNames = clParams.get(cl.name).toArray(new TypedParameter[0]);
                    for(int i=0; i<paramNames.length; i++) {
                        cl.param_names[i] = paramNames[i].getName();
                    }
                }

                CardLayout cardlayout = (CardLayout)(controlCards.getLayout());
                cardlayout.show(controlCards, cl.name);
            }
        }
    }

    class TestChangeListener implements ItemListener
    {
        public void itemStateChanged(ItemEvent event)
        {
            if (event.getStateChange() == ItemEvent.SELECTED) {
                ct.name = (String)event.getItem();
                ct.num_params = ctParams.get(ct.name).size();
                ct.param_names = new String[ct.num_params];
                ct.param_values = new typed_value_t[ct.num_params];
                if(ct.num_params > 0) {
                    TypedParameter[] paramNames = ctParams.get(ct.name).toArray(new TypedParameter[0]);
                    for(int i=0; i<paramNames.length; i++) {
                        ct.param_names[i] = paramNames[i].getName();
                    }
                }

                ct.compare_type = condition_test_t.CMP_GTE;
                ct.compared_value = TypedValue.wrap(0);

                CardLayout cardlayout = (CardLayout)(testCards.getLayout());
                cardlayout.show(testCards, ct.name);
            }
        }
    }

    public CommandSpoofer()
    {
        super("CommandSpoofer");
        this.setSize(600, 400);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));

        this.addComponentToPane(getContentPane());

        //Display the window.
        this.pack();
        this.setVisible(true);
    }


    public static void main(String[] args)
    {
        CommandSpoofer cs = new CommandSpoofer();
    }
}
