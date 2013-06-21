package probcog.classify;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import probcog.lcmtypes.category_t;
import probcog.perception.*;

public class Features
{
	public enum FeatureCategory
    {
		COLOR, SHAPE, SIZE, LOCATION
	}

	// Mapping from a FeatureCategory to the category_t enum
	private static HashMap<FeatureCategory, Integer> featureToLCMCat;
	static
    {
		featureToLCMCat = new HashMap<FeatureCategory, Integer>();
		featureToLCMCat.put(FeatureCategory.COLOR, category_t.CAT_COLOR);
		featureToLCMCat.put(FeatureCategory.SHAPE, category_t.CAT_SHAPE);
		featureToLCMCat.put(FeatureCategory.SIZE, category_t.CAT_SIZE);
		featureToLCMCat.put(FeatureCategory.LOCATION, category_t.CAT_LOCATION);
	}

	public static Integer getLCMCategory(FeatureCategory cat)
    {
		return featureToLCMCat.get(cat);
	}

	// Mapping from the LCM category_t.cat enum to a FeatureCategory
	private static HashMap<Integer, FeatureCategory> lcmToFeatureCat;
	static
    {
		lcmToFeatureCat = new HashMap<Integer, FeatureCategory>();
		lcmToFeatureCat.put(category_t.CAT_COLOR, FeatureCategory.COLOR);
		lcmToFeatureCat.put(category_t.CAT_SHAPE, FeatureCategory.SHAPE);
		lcmToFeatureCat.put(category_t.CAT_SIZE, FeatureCategory.SIZE);
		lcmToFeatureCat.put(category_t.CAT_LOCATION, FeatureCategory.LOCATION);
	}

	public static FeatureCategory getFeatureCategory(Integer lcmCat)
    {
		return lcmToFeatureCat.get(lcmCat);
	}

	public static ArrayList<Double> getFeatures(FeatureCategory cat,
			PointCloud cloud)
    {
		switch (cat) {
		case COLOR:
			return ColorFeatureExtractor.getFeatures(cloud);
		case SIZE:
			return SizeFeatureExtractor.getFeatures(cloud);
		case SHAPE:
			return ShapeFeatureExtractor.getFeatures(cloud);
		}
		return null;
	}
}
