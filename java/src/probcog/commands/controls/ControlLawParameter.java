package probcog.command.controls;

import probcog.command.*;

/** A parameter that can be fed to a control law.
 *
 *  Defines relevant ranges of values than can be assumed (if any), the
 *  type that is expected, whether or not this is a required parameter or
 *  optionally settable, etc.
 **/
public class ControlLawParameter
{
    private public String name;     // What is this parameter called?
    private boolean required;       // It it required (true) or optional (false) for construction?

    private int type;   // XXX Pulls type from TypedValue?
    private TypedValue[] range = new TypedValue[2]; // Min to max

    public ControlLawParameter(String name, int type)
    {
        this.name = name;
        this.type = type;
    }

    public ControlLawParameter(String name, int type, TypedValue min, TypedValue max)
    {
        assert (type == min.getType() && type == max.getType());

        this.name = name;
        this.type = type;
        range[0] = min;
        range[1] = max;
    }

    public String getName()
    {
        return name;
    }

    public boolean isRequired()
    {
        return required;
    }

    public int getType()
    {
        return type;
    }

    public boolean hasRange()
    {
        return range[0] != null && range[1] != null;
    }

    public TypedValue[] getRange() throws UnsupportedOperationException
    {
        if (hasRange())
            return range;
        else
            throw new UnsupportedOperationException("No range specified");
    }
}
