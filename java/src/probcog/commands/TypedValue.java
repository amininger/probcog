package probcog.commands;

import probcog.lcmtypes.typed_value_t;

public class TypedValue{
	public static Double unwrapDouble(typed_value_t value){
		if(value.type == typed_value_t.TYPE_DOUBLE){
			return Double.parseDouble(value.value);
		} else {
			return null;
		}
	}

	public static Integer unwrapInt(typed_value_t value){
		if(value.type == typed_value_t.TYPE_INT){
			return Integer.parseInt(value.value);
		} else {
			return null;
		}
	}

	public static String unwrapString(typed_value_t value){
		if(value.type == typed_value_t.TYPE_STRING){
			return value.value;
		} else {
			return null;
		}
	}

	public static Boolean unwrapBoolean(typed_value_t value){
		if(value.type == typed_value_t.TYPE_BOOL){
			return new Boolean(value.value);
		} else {
			return null;
		}
	}

	public static typed_value_t wrap(Integer value){
		typed_value_t tv = new typed_value_t();
		tv.type = typed_value_t.TYPE_INT;
		tv.value = value.toString();
		return tv;
	}

	public static typed_value_t wrap(Double value){
		typed_value_t tv = new typed_value_t();
		tv.type = typed_value_t.TYPE_DOUBLE;
		tv.value = value.toString();
		return tv;
	}

	public static typed_value_t wrap(String value){
		typed_value_t tv = new typed_value_t();
		tv.type = typed_value_t.TYPE_STRING;
		tv.value = value;
		return tv;
	}

	public static typed_value_t wrap(Boolean value){
		typed_value_t tv = new typed_value_t();
		tv.type = typed_value_t.TYPE_BOOL;
		tv.value = value.toString();
		return tv;
	}

}
