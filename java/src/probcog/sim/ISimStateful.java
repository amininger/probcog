package probcog.sim;

public interface ISimStateful {
	
	// Gets the value of the provided state (or null if it doesn't exist)
	public String getState(String stateName);
	
	// Sets the state to the given value 
	public void setState(String stateName, String stateVal);
	
	// Gets the current state of the object in an array of 
	// Key/value arrays (i.e. [stateName, stateValue])
	public String[][] getCurrentState();
}
