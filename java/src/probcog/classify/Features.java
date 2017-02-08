package probcog.classify;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import probcog.perception.*;
import probcog.rosie.perception.CategorizedData.CategoryType;

public class Features
{
	public enum FeatureCategory
    {
		COLOR, SHAPE, SIZE, LOCATION, WEIGHT, TEMPERATURE
	}

	// Mapping from a FeatureCategory to the category_t enum
	private static HashMap<FeatureCategory, CategoryType> featureToCat;
	static
    {
		featureToCat = new HashMap<FeatureCategory, CategoryType>();
		featureToCat.put(FeatureCategory.COLOR, CategoryType.CAT_COLOR);
		featureToCat.put(FeatureCategory.SHAPE, CategoryType.CAT_SHAPE);
		featureToCat.put(FeatureCategory.SIZE, CategoryType.CAT_SIZE);
		featureToCat.put(FeatureCategory.LOCATION, CategoryType.CAT_LOCATION);
		featureToCat.put(FeatureCategory.WEIGHT, CategoryType.CAT_WEIGHT);
		featureToCat.put(FeatureCategory.TEMPERATURE, CategoryType.CAT_TEMPERATURE);
	}

	public static CategoryType getCategory(FeatureCategory cat)
    {
		return featureToCat.get(cat);
	}

	// Mapping from the enum to a FeatureCategory
	private static HashMap<CategoryType, FeatureCategory> catTypeToFeatureCat;
	static
    {
		catTypeToFeatureCat = new HashMap<CategoryType, FeatureCategory>();
		catTypeToFeatureCat.put(CategoryType.CAT_COLOR, FeatureCategory.COLOR);
		catTypeToFeatureCat.put(CategoryType.CAT_SHAPE, FeatureCategory.SHAPE);
		catTypeToFeatureCat.put(CategoryType.CAT_SIZE, FeatureCategory.SIZE);
		catTypeToFeatureCat.put(CategoryType.CAT_LOCATION, FeatureCategory.LOCATION);
		catTypeToFeatureCat.put(CategoryType.CAT_WEIGHT, FeatureCategory.WEIGHT);
		catTypeToFeatureCat.put(CategoryType.CAT_TEMPERATURE, FeatureCategory.TEMPERATURE);
	}

	public static FeatureCategory getFeatureCategory(CategoryType catType)
    {
		return catTypeToFeatureCat.get(catType);
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
