package probcog.commands;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import javax.swing.*;
import javax.swing.event.*;

import probcog.commands.controls.*;
import probcog.commands.tests.*;
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
    // public static control_law_t createControlLaw(String clName,
    //                                              String testName,
    //                                              ArrayList<String> params)
    // {
    //     control_law_t cl = new control_law_t();
    //     cl.id = 1;
    //     cl.name = clName;
    //     cl.num_params = 0;
    //     cl.param_names = new String[0];
    //     cl.param_values = new typed_value_t[0];

    //     double value = Double.parseDouble(params.get(0));

    //     if(clName.equals("turn"))
    //     {
    //         cl.num_params = 1;
    //         cl.param_names = new String[]{"direction"};
    //         cl.param_values = new typed_value_t[1];
    //         if(value < 0)
    //             cl.param_values[0] = TypedValue.wrap(-1);
    //         else
    //             cl.param_values[0] = TypedValue.wrap(1);
    //     }

    //     if ("follow-wall".equals(clName)) {
    //         cl.num_params = 2;
    //         cl.param_names = new String[]{"side", "distance"};
    //         cl.param_values = new typed_value_t[2];
    //         if(value < 0)
    //             cl.param_values[0] = TypedValue.wrap(-1);
    //         else
    //             cl.param_values[0] = TypedValue.wrap(1);
    //         cl.param_values[1] = TypedValue.wrap(value);
    //     }

    //     if ("follow-heading".equals(clName)) {
    //         cl.num_params = 1;
    //         cl.param_names = new String[]{"heading"};
    //         cl.param_values = new typed_value_t[1];
    //         cl.param_values[0] = TypedValue.wrap(MathUtil.mod2pi(value));
    //     }

    //     condition_test_t ct = new condition_test_t();
    //     ct.name = testName;
    //     ct.num_params = 0;
    //     ct.param_names = new String[0];
    //     ct.param_values = new typed_value_t[0];
    //     ct.compare_type = condition_test_t.CMP_GTE;
    //     if ("follow-wall".equals(clName) || "follow-heading".equals(clName)) {
    //         ct.compared_value = TypedValue.wrap(1000.0); // Infinity
    //     } else {
    //         ct.compared_value = TypedValue.wrap(value);
    //     }

    //     if(clName.equals("turn") && value<0)
    //     {
    //         ct.compare_type = condition_test_t.CMP_LTE;
    //     }

    //     if ("count".equals(testName)) {
    //         ct.num_params = 2;
    //         ct.param_names = new String[]{"count", "class"};
    //         ct.param_values = new typed_value_t[2];
    //         ct.param_values[0] = TypedValue.wrap(value);
    //         ct.param_values[1] = TypedValue.wrap(params.get(1));
    //     }

    //     cl.termination_condition = ct;
    //     return cl;
    // }

    JPanel commandParamPanel, testParamPanel;
    JList commandList, testList;

    Map<String, Collection<TypedParameter>> clParams, ctParams;

    // public CommandSpoofer()
    // {
    //     super("CommandSpoofer");
    //     this.setSize(600, 400);
    //     this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    //     setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));

    //     ControlLawFactory clfactory = ControlLawFactory.getSingleton();
    //     ConditionTestFactory ctfactory = ConditionTestFactory.getSingleton();
    //     clParams = clfactory.getParameters();
    //     ctParams = ctfactory.getParameters();

    //     JPanel commandPanel = new JPanel(new BorderLayout());
    //     JPanel testPanel = new JPanel(new BorderLayout());
    //     commandParamPanel = new JPanel();
    //     testParamPanel = new JPanel();

    //     JLabel commandLabel = new JLabel("Commands",JLabel.LEFT);
    //     JLabel testLabel = new JLabel("Tests",JLabel.LEFT);

    //     commandList = new JList(clParams.keySet().toArray(new String[0]));
    //     commandList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
    //     commandList.getSelectionModel().addListSelectionListener( new CommandListSelectionHandler());

    //     testList = new JList(ctParams.keySet().toArray(new String[0]));
    //     testList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);

    //     commandPanel.add(commandLabel, BorderLayout.NORTH);
    //     commandPanel.add(commandList, BorderLayout.CENTER);

    //     testPanel.add(testLabel, BorderLayout.NORTH);
    //     testPanel.add(testList, BorderLayout.CENTER);


    //     // JButton command = new JButton("Command");
    //     // command.addActionListener(new CommandListener());

    //     this.add(commandPanel);
    //     this.add(commandParamPanel);
    //     this.add(testPanel);
    //     this.add(testParamPanel);
    //     this.setVisible(true);
    // }



    // private class CommandListSelectionHandler implements ListSelectionListener
    // {
    //     public void valueChanged(ListSelectionEvent listSelectionEvent)
    //     {
    //         ListSelectionModel lsm = (ListSelectionModel) listSelectionEvent.getSource();
    //         if (!listSelectionEvent.getValueIsAdjusting() && !lsm.isSelectionEmpty()) {

    //             // commandParamPanel = new JPanel();

    //             commandParamPanel.add(new JLabel("Parameters:"));
    //             commandParamPanel.revalidate();
    //             commandParamPanel.repaint();

    //             String key = (String) commandList.getSelectedValue();
    //             Collection<TypedParameter> params = clParams.get(key);

    //             for(TypedParameter tp : params)
    //             {
    //                 String label = tp.getName();
    //                 if(tp.hasValid()) {
    //                     Collection<TypedValue> valid = tp.getValid();
    //                     label += " (";
    //                     for(TypedValue v : valid) {
    //                         label += " "+v.toString();
    //                     }
    //                     label += "):";
    //                 }
    //                 JLabel lab = new JLabel(label);
    //                 commandParamPanel.add(lab);
    //                 System.out.println(tp.getName());
    //             }
    //         }
    //     }
    // }



    JPanel controlCards, testCards;

    public void addComponentToPane(Container pane)
    {
        // Get control laws and termination conditions
        ControlLawFactory clfactory = ControlLawFactory.getSingleton();
        ConditionTestFactory ctfactory = ConditionTestFactory.getSingleton();
        clParams = clfactory.getParameters();
        ctParams = ctfactory.getParameters();

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
            JPanel card = createPanel(clParams.get(law));
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
            JPanel card = createPanel(ctParams.get(test));
            testCards.add(card, test);
        }

        // Create button for sending the lcm message
        JPanel buttonPane = new JPanel();
        JButton sendButton = new JButton("Send Command");
        buttonPane.add(sendButton);

        // Add everything to our panel
        pane.add(controlPane);
        pane.add(controlCards);
        pane.add(testPane);
        pane.add(testCards);
        pane.add(buttonPane);
    }

    private JPanel createPanel(Collection<TypedParameter> params)
    {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        // panel.add(new JLabel(name, JLabel.LEFT));

        for(TypedParameter tp : params)
        {
            JPanel jp = new JPanel();
            String paramName = tp.getName();
            String type = typeToString(tp.getType());

            jp.add(new JLabel(paramName, JLabel.LEFT));
            System.out.println(paramName+" "+type);

            if(tp.hasValid()) {
                Collection<TypedValue> valid = tp.getValid();
                for(TypedValue v : valid) {
                }
            }
            else if(tp.hasRange()) {

            }
            else {

            }

            panel.add(jp);
        }


        return panel;
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

    class ControlChangeListener implements ItemListener
    {
        public void itemStateChanged(ItemEvent event) {
            if (event.getStateChange() == ItemEvent.SELECTED) {
                CardLayout cl = (CardLayout)(controlCards.getLayout());
                cl.show(controlCards, (String)event.getItem());
            }
        }
    }

    class TestChangeListener implements ItemListener
    {
        public void itemStateChanged(ItemEvent event) {
            if (event.getStateChange() == ItemEvent.SELECTED) {
                CardLayout cl = (CardLayout)(testCards.getLayout());
                cl.show(testCards, (String)event.getItem());
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

    //     if(args.length < 3){
    //         System.err.println("Need at least 3 args: [command name] [test condition] [parameters]");
    //         System.err.println("Possible command types incluede:");
    //         System.err.println("\tdrive-forward distance [# meters]");
    //         System.err.println("\tdrive-forward count [number] [things-to-count]");
    //         System.err.println("\tturn rotation [# rads]");
    //         System.err.println("\tfollow-wall distance [# meters from wall (sign indicates which wall)]");
    //         System.err.println("\tfollow-heading distance [heading rads]");
    //     }

    //     String clName = args[0];
    //     String ctName = args[1];
    //     ArrayList<String> params = new ArrayList<String>();
    //     for(int i=2; i<args.length; i++) {
    //         params.add(args[i]);
    //     }

    //     control_law_t cl = createControlLaw(clName, ctName, params);

    //     LCM lcm = LCM.getSingleton();
    //     lcm.publish("SOAR_COMMAND", cl);
    }
}
