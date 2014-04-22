package probcog.commands.tests;

import probcog.lcmtypes.condition_test_t;

// XXX Will probably want to change this, too...
public class ConditionTestFactory{
	public static IConditionTest construct(condition_test_t test){
		if(test.name.equals("distance")){
			return new DistanceTest(test);
		} else if(test.name.equals("rotation")){
			return new RotationTest(test);
		} else if(test.name.equals("obstruction")){
			return new ObstructionTest(test);
		}

		return new NullTest(test);
	}
}
