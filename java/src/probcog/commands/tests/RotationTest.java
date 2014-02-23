package probcog.commands.tests;

import probcog.lcmtypes.condition_test_t;
import probcog.lcmtypes.typed_value_t;

import probcog.commands.TypedValue;

public class RotationTest extends ConditionTest<Double>{

	public RotationTest(condition_test_t test){
		super(test);
	}

	@Override
	protected Double getTarget(typed_value_t value){
		return TypedValue.unwrapDouble(value);
	}

	@Override
	protected Double getValue(){
		// TODO: Return the cumulative amount the robot has rotated in radians
		//   positive for turning left
		//   negative for turning right
		return 0.0;
	}
}
