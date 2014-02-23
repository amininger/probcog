package probcog.commands.tests;

import probcog.lcmtypes.condition_test_t;
import probcog.lcmtypes.typed_value_t;

import probcog.commands.TypedValue;

public class DistanceTest extends ConditionTest<Double>{

	public DistanceTest(condition_test_t test){
		super(test);
	}

	@Override
	protected Double getTarget(typed_value_t value){
		return TypedValue.unwrapDouble(value);
	}

	@Override
	protected Double getValue(){
		// TODO: Return the distance traveled by the robot so far
		return 0.0;
	}
}
