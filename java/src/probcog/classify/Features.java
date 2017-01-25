package probcog.classify;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

//import probcog.lcmtypes.category_t;
import probcog.perception.*;
import probcog.rosie.perception.CategorizedData.CategoryType;

public class Features
{
	public enum FeatureCategory
    {
		COLOR, SHAPE, SIZE, LOCATION, WEIGHT, TEMPERATURE
	}

	// Mapping from a FeatureCategory to the category_t enum
	private static HashMap<FeatureCategory, CategoryType> featureToLCMCat;
	static
    {
		featureToLCMCat = new HashMap<FeatureCategory, CategoryType>();
		featureToLCMCat.put(FeatureCategory.COLOR, CategoryType.CAT_COLOR);
		featureToLCMCat.put(FeatureCategory.SHAPE, CategoryType.CAT_SHAPE);
		featureToLCMCat.put(FeatureCategory.SIZE, CategoryType.CAT_SIZE);
		featureToLCMCat.put(FeatureCategory.LOCATION, CategoryType.CAT_LOCATION);
		featureToLCMCat.put(FeatureCategory.WEIGHT, CategoryType.CAT_WEIGHT);
		featureToLCMCat.put(FeatureCategory.TEMPERATURE, CategoryType.CAT_TEMPERATURE);
	}

	public static CategoryType getLCMCategory(FeatureCategory cat)
    {
		return featureToLCMCat.get(cat);
	}

	// Mapping from the LCM category_t.cat enum to a FeatureCategory
	private static HashMap<CategoryType, FeatureCategory> lcmToFeatureCat;
	static
    {
		lcmToFeatureCat = new HashMap<CategoryType, FeatureCategory>();
		lcmToFeatureCat.put(CategoryType.CAT_COLOR, FeatureCategory.COLOR);
		lcmToFeatureCat.put(CategoryType.CAT_SHAPE, FeatureCategory.SHAPE);
		lcmToFeatureCat.put(CategoryType.CAT_SIZE, FeatureCategory.SIZE);
		lcmToFeatureCat.put(CategoryType.CAT_LOCATION, FeatureCategory.LOCATION);
		lcmToFeatureCat.put(CategoryType.CAT_WEIGHT, FeatureCategory.WEIGHT);
		lcmToFeatureCat.put(CategoryType.CAT_TEMPERATURE, FeatureCategory.TEMPERATURE);
	}

	public static FeatureCategory getFeatureCategory(CategoryType lcmCat)
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
		if(cloud.getPoints().size() == 0){
			return null;
		}
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
