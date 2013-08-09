package probcog.classify;

import java.util.*;

public interface Classifier
{
    /** Add an existing point to the classifier */
    void add(CPoint point);

    /** Remove and return the last point, if it exists */
    CPoint removeLast();

	/**
	 * @param object
	 * @return the label for the object, or "unknown" if not found
	 */
    Classifications classify(ArrayList<Double> features);


	/**
	 * Clears all data in the classifier
	 */
	void clearData();


	/**
	 * Reloads all the data in the classifier
	 */
	void loadData();
}
