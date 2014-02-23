package probcog.commands.tests;

import probcog.lcmtypes.condition_test_t;
import probcog.lcmtypes.typed_value_t;

import probcog.commands.TypedValue;

import april.util.TimeUtil;

public class ObstructionTest extends ConditionTest<Boolean>{
	long finishTime;

	public ObstructionTest(condition_test_t test){
		super(test);

		finishTime = TimeUtil.utime() + 3000000;
	}

	@Override
	protected Boolean getTarget(typed_value_t value){
		return TypedValue.unwrapBoolean(value);
	}

	@Override
	protected Boolean getValue(){
		// TODO: Returns true if there is an obstruction in front of the robot
		if(TimeUtil.utime() > finishTime){
			return true;
		}
		return false;
	}
}
