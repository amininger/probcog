package probcog.commands.controls;

import probcog.lcmtypes.control_law_t;

import probcog.commands.tests.IConditionTest;
import probcog.commands.tests.ConditionTestFactory;

public abstract class ControlLaw
{
	protected String name;
	protected int id;

	protected IConditionTest termCond;

	public enum Status
    {
		EXECUTING, FINISHED, EARLY_TERM, FAILURE
	}

	public ControlLaw(control_law_t controlLaw)
    {
		name = controlLaw.name;
		id = controlLaw.id;

		termCond = ConditionTestFactory.construct(controlLaw.termination_condition);
	}

	public int getID()
    {
		return id;
	}

	public String getName()
    {
		return name;
	}

	public abstract void execute();

	public abstract Status getStatus();

}
