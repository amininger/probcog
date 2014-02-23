package probcog.rosie.script;

import probcog.rosie.testing.Settings;

public class AutomateScript implements UiCommand {
	boolean automated;
	
	public AutomateScript(boolean isAutomated) {
		automated = isAutomated;
	}
	@Override
	public void execute() {
		Settings.getInstance().setAutomated(automated);
	}
}
