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
		COLOR, SHAPE, SIZE, LOCATION, WEIGHT, TEMPERATURE
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
		featureToLCMCat.put(FeatureCategory.WEIGHT, category_t.CAT_WEIGHT);
		featureToLCMCat.put(FeatureCategory.TEMPERATURE, category_t.CAT_TEMPERATURE);
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
		lcmToFeatureCat.put(category_t.CAT_WEIGHT, FeatureCategory.WEIGHT);
		lcmToFeatureCat.put(category_t.CAT_TEMPERATURE, FeatureCategory.TEMPERATURE);
	}

	public static FeatureCategory getFeatureCategory(Integer lcmCat)
    {
		return lcmToFeatureCat.get(lcmCat);
	}
	
	// Mapping from strings to FeatureCategory
	private static HashMap<String, FeatureCategory> nameToFeatureCat;
	static{
		nameToFeatureCat = new HashMap<String, FeatureCategory>();
		nameToFeatureCat.put("color", FeatureCategory.COLOR);
		nameToFeatureCat.put("shape", FeatureCategory.SHAPE);
		nameToFeatureCat.put("size", FeatureCategory.SIZE);
		nameToFeatureCat.put("weight", FeatureCategory.WEIGHT);
		nameToFeatureCat.put("temperature", FeatureCategory.TEMPERATURE);
	}
	public static FeatureCategory getFeatureCategory(String catName){
		return nameToFeatureCat.get(catName);
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
		case WEIGHT:
			return WeightFeatureExtractor.getFeatures(cloud);
		case TEMPERATURE:
			return TemperatureFeatureExtractor.getFeatures(cloud);
		}
		return null;
	}
	
	public static boolean isVisualFeature(FeatureCategory cat){
		switch(cat){
		case COLOR:
		case SIZE:
		case SHAPE:
			return true;
		default:
			return false;
		}
	}
}
