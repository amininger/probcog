package probcog.commands.tests;

import probcog.lcmtypes.condition_test_t;
import probcog.lcmtypes.typed_value_t;

import probcog.commands.TypedValue;

import april.util.TimeUtil;

public class ObstructionTest extends ConditionTest<Boolean>{
	public ObstructionTest(condition_test_t test){
		super(test);

	}

	@Override
	protected Boolean getTarget(typed_value_t value){
		return TypedValue.unwrapBoolean(value);
	}

	@Override
	protected Boolean getValue(){
		// TODO: Returns true if there is an obstruction in front of the robot
		return false;
	}
}
