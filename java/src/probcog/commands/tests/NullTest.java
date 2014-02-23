package probcog.commands.tests;

import probcog.lcmtypes.condition_test_t;
import probcog.lcmtypes.typed_value_t;

import probcog.commands.TypedValue;

public class NullTest extends ConditionTest<Double>{

	public NullTest(condition_test_t test){
		super(test);
	}

	@Override
	public boolean evaluate(){
		return false;
	}

	@Override
	protected Double getTarget(typed_value_t value){
		return 0.0;
	}

	@Override
	protected Double getValue(){
		return 0.0;
	}
}
