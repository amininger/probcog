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

    /**
     * Create a panel that has all the control laws and termination conditions
     * that are registered and displays what parameters they take.
     **/
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
        for(int i=0; i<controlLaws.length; i++) {
            String law = controlLaws[i];
            JPanel card = createPanel(clParams.get(law), COMMAND);
            controlCards.add(card, law);
        }
        initializeControlLCM(controlLaws[0]);


        // Create panel for the termination tests
        JPanel testPane = new JPanel(); //use FlowLayout
        String[] tests = ctParams.keySet().toArray(new String[0]);
        JComboBox testCombo = new JComboBox(tests);
        testCombo.setEditable(false);
        testCombo.addItemListener(new TestChangeListener());
        testPane.add(testCombo);

        // Create the panel that contains the termination test "cards" and
        // create those cards
        testCards = new JPanel(new CardLayout());
        for(int i=0; i<tests.length; i++) {
            String test = tests[i];
            JPanel card = createPanel(ctParams.get(test), TEST);
            testCards.add(card, test);
        }
        initializeTestLCM(tests[0]);

        // Create button for sending the lcm message
        JPanel buttonPane = new JPanel();
        buttonPane.setLayout(new BoxLayout(buttonPane, BoxLayout.Y_AXIS));
        JButton sendButton = new JButton("Send Command");
        sendButton.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    publishControlLaw();
                }
            });
        buttonPane.add(sendButton);

        // XXX -- Doesn't actually follow through in the command coordinator
        // Create button for stopping lcm message
        JButton stopButton = new JButton("Stop Command");
        stopButton.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    publishStop();
                }
            });
        buttonPane.add(stopButton);

        // Add everything to our panel
        pane.add(controlPane);
        pane.add(controlCards);
        pane.add(testPane);
        pane.add(testCards);
        pane.add(buttonPane);
    }

    /**
     * A panel is created for each control law and test condition, these are shown
     * when their corresponding law/test is selected. Each one shows the parameters
     * available to be changed (including ranges and valid selections if those exist
     * and alerts the user as to what type is expected.
     **/
    private JPanel createPanel(Collection<TypedParameter> params, String cardType)
    {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        TypedParameter[] paramsArray = params.toArray(new TypedParameter[0]);

        // Each parameter that can be changed gets its own line
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

            // If there are only a few options that can be selected, display them
            // in a combo box.
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
                // If a range of options is available, tell the user what it is
                // DOES NOT PERFORM CHECK!
                if(tp.hasRange()) {
                    TypedValue[] range = tp.getRange();
                    String rangeLabel = "("+range[0].toString()+", "+range[1].toString()+")";
                    jp.add(new JLabel(rangeLabel));
                }
                // Let user know what type is expected
                else {
                    String typeLabel = "("+type+")";
                    jp.add(new JLabel(typeLabel));
                }

                // Update the lcm message every time a key is entered or removed
                // from the box
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

    /**
     * Transform the given string input into an lcmtype that has the
     * correct typing (i.e. int, string, double, boolean)
     **/
    private typed_value_t getTypedValueLCM(int type, String input)
    {
        if(input.isEmpty() || input == null)
            input = "0";
        try {
            switch (type) {
                case TypedValue.TYPE_BOOLEAN:
                    if(input.isEmpty() || input == "0")
                        return TypedValue.wrap(false);
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
                    if(input.isEmpty() || input == "0")
                        TypedValue.wrap("");
                    return TypedValue.wrap(input);
            }
        }
        catch(Exception ex) {
            ex.printStackTrace();
        }

        return new typed_value_t();
    }

    /**
     * @return a string describing the primitive type
     **/
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

    private void initializeControlLCM(String name)
    {
        cl.name = name;
        cl.num_params = clParams.get(cl.name).size();
        cl.param_names = new String[cl.num_params];
        cl.param_values = new typed_value_t[cl.num_params];

        // If necessary, initialize param names and values
        if(cl.num_params > 0) {
            TypedParameter[] paramNames = clParams.get(cl.name).toArray(new TypedParameter[0]);
            for(int i=0; i<cl.num_params; i++) {
                TypedParameter tp = paramNames[i];

                cl.param_names[i] = tp.getName();

                TypedValue value = new TypedValue("");
                if(tp.hasValid()) {
                    value =  tp.getValid().toArray(new TypedValue[0])[0];
                }

                cl.param_values[i] = getTypedValueLCM(tp.getType(), value.toString());
            }
        }
    }

    private void initializeTestLCM(String name)
    {
        ct.name = name;
        ct.num_params = ctParams.get(ct.name).size();
        ct.param_names = new String[ct.num_params];
        ct.param_values = new typed_value_t[ct.num_params];

        // If necessary, initialize param names and values
        if(ct.num_params > 0) {
            TypedParameter[] paramNames = ctParams.get(ct.name).toArray(new TypedParameter[0]);
            for(int i=0; i<ct.num_params; i++) {
                TypedParameter tp = paramNames[i];

                ct.param_names[i] = tp.getName();

                TypedValue value = new TypedValue("");
                if(tp.hasValid()) {
                    value =  tp.getValid().toArray(new TypedValue[0])[0];
                }

                ct.param_values[i] = getTypedValueLCM(tp.getType(), value.toString());
            }
        }

        ct.compare_type = condition_test_t.CMP_GTE;
        ct.compared_value = TypedValue.wrap(0);
    }


    /**
     * Publish control laws when the button is pressed. The method assumes
     * that the information has been correctly entered into the control_law
     * and test_condition lcm messages correctly and only checks that they
     * exist.
     **/
    private static int id = 0;
    private String lastControl;
    private void publishControlLaw()
    {
        if(cl == null || ct == null)
            return;

        cl.id = id;
        id ++;
        lastControl = cl.name;

        cl.utime = TimeUtil.utime();
        cl.termination_condition = ct;
        lcm.publish("SOAR_COMMAND", cl);
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

        lcm.publish("CONTROL_LAW_STATUS", status_list);
    }

    /**
     * Listens for changes in the control law combo box. Whenever a new control law
     * is selected, the control_law lcm is updated with the correct name, parameters,
     * etc and the correct JPanel is displayed for editing.
     **/
    class ControlChangeListener implements ItemListener
    {
        public void itemStateChanged(ItemEvent event)
        {
            if (event.getStateChange() == ItemEvent.SELECTED) {

                initializeControlLCM((String) event.getItem());
                CardLayout cardlayout = (CardLayout)(controlCards.getLayout());
                cardlayout.show(controlCards, cl.name);
            }
        }
    }

    /**
     * Listens for changes in the test condition combo box. Whenever a new test condition
     * is selected, the condition_test lcm is updated with the correct name, parameters,
     * etc and the correct JPanel is displayed for editing.
     **/
    class TestChangeListener implements ItemListener
    {
        public void itemStateChanged(ItemEvent event)
        {
            if (event.getStateChange() == ItemEvent.SELECTED) {
                initializeTestLCM((String)event.getItem());
                CardLayout cardlayout = (CardLayout)(testCards.getLayout());
                cardlayout.show(testCards, ct.name);
            }
        }
    }

    public static void main(String[] args)
    {
        CommandSpoofer cs = new CommandSpoofer();
    }
}
