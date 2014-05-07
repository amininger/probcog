package probcog.commands.controls;

import java.util.*;
import java.lang.reflect.InvocationTargetException;

import probcog.commands.*;
import probcog.lcmtypes.*;

public class ControlLawFactory
{
    // Control law factory: take 2
    public static construct(HashMap<String, Object> parameters)


    static Object lock = new Object();
    static public Object getLock()
    {
        return lock;
    }

    static HashMap<String, String> controlLawMap = new HashMap<String, String>();
    static boolean initialized = false;

    /** Initializes the factory on first use. This is where the default
     *  registrations are added.
     **/
    static void init()
    {
        registerControlLaw("turn", Turn.class.getName());
        registerControlLaw("drive-forward", DriveForward.class.getName());

        initialized = true;
    }

    /** Register control laws with the factory. It is the job of the
     *  writer to ensure that provided classes implement the appropriate
     *  interface. Reflection is used to construct an instance of the
     *  appropriate class.
     *
     *  @param name         The name provided by the external system
     *  @param classname    The given name of the class.
     *
     **/
    public static void registerControlLaw(String name, String classname)
    {
        synchronized (getLock()) {
            controlLawMap.put(name, classname);
        }
    }

    /** Unregister a control law. Returns true/false if control law was
     *  successfully removed/no matching key was found.
     *
     *  @param name         The name of the control law to be removed
     *
     *  @return True if control law successfully removed, else false
     **/
    public static boolean unregisterControllaw(String name)
    {
        String val;
        synchronized (getLock()) {
            val = controlLawMap.remove(name);
        }
        return (val != null);
    }

    /** Get the names of all registered control laws. These differ from class
     *  names, which actually specify the particular implementation class of
     *  the control law.
     *
     *  @return A set of the externally used names of registered control laws
     **/

    public static Set<String> getControlLawNames()
    {
        Set<String> keys;
        synchronized (getLock()) {
             keys = controlLawMap.keySet();
        }
        return keys;
    }

    /** Control a control law from an associated lcmtype specifying a name
     *  and relevant parameters.
     *
     *  @param controlLaw   An LCM object containing relevant information to
     *                      instatiate a control law.
     *
     *  @return A ControlLaw object that a robot may execute
     **/
	public static ControlLaw construct(control_law_t controlLaw) throws ClassNotFoundException
    {
        // First time initialization
        synchronized (getLock()) {
            if (!initialized)
                init();

            // Ensure class existence
            if (!controlLawMap.containsKey(controlLaw.name)) {
                throw new ClassNotFoundException(controlLaw.name);
            }

            // Instantiate the appropriate control law
            try {
                String classname = controlLawMap.get(controlLaw.name);
                Object obj = Class.forName(classname).getConstructor(control_law_t.class).newInstance(controlLaw);
                assert (obj instanceof ControlLaw);

                return (ControlLaw) obj;
            } catch (InvocationTargetException ex) {
                System.err.printf("ERR: %s (%s)\n", ex, ex.getTargetException());
            } catch (Exception ex) {
                System.err.printf("ERR: %s\n", ex);
                ex.printStackTrace();
            }
        }
        return null;    // XXX
	}

    public static void main(String[] args)
    {
        // Test the ControlLawFactory's functionality. By no means complete,
        // yet, but enough to prove that the implementation is not completely
        // flawed.
        control_law_t cl = new control_law_t();
        cl.name = "turn";
        cl.id = 0;

        cl.num_params = 1;
        cl.param_names = new String[] {"direction"};
        cl.param_values = new typed_value_t[] {TypedValue.wrap("right")};

        // XXX This is not intended to be accurate or useful. We're just making sure
        // we can appropriately act on control laws, etc.
        cl.termination_condition = new condition_test_t();
        cl.termination_condition.name = "rotation"; // XXX
        cl.termination_condition.num_params = 0;
        cl.termination_condition.compare_type = condition_test_t.CMP_LTE;   // XXX
        cl.termination_condition.compared_value = TypedValue.wrap(Math.PI/2.0);


        try {
            ControlLaw law = ControlLawFactory.construct(cl);
            System.out.printf("[%s] %d: %s\n",
                              law.getName(),
                              law.getID(),
                              law.getStatus().name());
            // Automatically spins up termination condition test thread...

            Set<String> names = ControlLawFactory.getControlLawNames();
            for (String name: names)
                System.out.println(name);


        } catch (ClassNotFoundException ex) {
            System.err.println("ERR: Could not construct control law" + ex);
            ex.printStackTrace();
        }
    }
}
