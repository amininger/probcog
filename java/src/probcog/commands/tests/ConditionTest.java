package probcog.commands.tests;

import probcog.lcmtypes.condition_test_t;
import probcog.lcmtypes.typed_value_t;

public abstract class ConditionTest<T extends Comparable> implements IConditionTest{
	protected int compareType;
	protected T targetValue;

	public ConditionTest(condition_test_t test){
		compareType = test.compare_type;
		targetValue = getTarget(test.compared_value);
	}

	// The target value which is being compared against
	protected abstract T getTarget(typed_value_t value);

	// The value which is re-evaluated and compared to the target
	//   to see if it meets the desired condition
	protected abstract T getValue();

	// Returns true if the test is true 
	//   Re-evaluates the condition and compares against the target
	public boolean evaluate(){
		T value = getValue();
		switch(compareType){
			case condition_test_t.CMP_GT:
				return value.compareTo(targetValue) > 0 ;
			case condition_test_t.CMP_GTE:
				return value.compareTo(targetValue) >= 0 ;
			case condition_test_t.CMP_EQ:
				return value.compareTo(targetValue) == 0 ;
			case condition_test_t.CMP_LTE:
				return value.compareTo(targetValue) <= 0 ;
			case condition_test_t.CMP_LT:
				return value.compareTo(targetValue) < 0 ;
			default:
				// Error case: never get here
				return true;
		}
	}

}
