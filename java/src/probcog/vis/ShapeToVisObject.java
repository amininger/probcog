package probcog.vis;

import java.util.ArrayList;

import april.jmat.LinAlg;
import april.sim.*;
import april.vis.*;

public class ShapeToVisObject
{
	public static ArrayList<VisObject> getVisObjects(Shape shape, VzMesh.Style style)
    {
		if(shape instanceof SphereShape)
			return getVisObjects((SphereShape)shape, style);
		// if(shape instanceof BoxShape)
		// 	return getVisObjects((BoxShape)shape, style);
		// if(shape instanceof ConvexShape)
		// 	return getVisObjects((ConvexShape)shape, style);
		// if(shape instanceof CompoundShape)
		// 	return getVisObjects((CompoundShape)shape, style);
		return null;
	}

	public static ArrayList<VisObject> getVisObjects(SphereShape shape, VzMesh.Style style)
    {
		ArrayList<VisObject> objs = new ArrayList<VisObject>();
		objs.add(new VisChain(LinAlg.scale(shape.getRadius()), new VzSphere(style)));
		return objs;
	}

    // XXX -- Currently removed boxShapes, convexShapes and compound shapes
	// public static ArrayList<VisObject> getVisObjects(BoxShape shape, VzMesh.Style style)
    // {
	// 	ArrayList<VisObject> objs = new ArrayList<VisObject>();
	// 	objs.add(new VisChain(LinAlg.scale(shape.sxyz[0], shape.sxyz[1], shape.sxyz[2]), new VzBox(style)));
	// 	return objs;
	// }

	// public static ArrayList<VisObject> getVisObjects(ConvexShape shape, VzMesh.Style style)
    // {
	// 	ArrayList<VisObject> objs = new ArrayList<VisObject>();
	// 	objs.add(new VzMesh(shape.getVertexData(), shape.getNormals(), VzMesh.TRIANGLES, style));
	// 	return objs;
	// }

	// public static ArrayList<VisObject> getVisObjects(CompoundShape shape, VzMesh.Style style)
    // {
	// 	ArrayList<VisObject> objs = new ArrayList<VisObject>();

	// 	double[][] xform = LinAlg.identity(4);
	// 	for(Object op : shape.ops){
	// 		if(op instanceof double[][]){
	// 			LinAlg.timesEquals(xform,  (double[][])op);
	// 		} else if(op instanceof Shape){
	// 			ArrayList<VisObject> visObjs = getVisObjects((Shape)op, style);
	// 			for(VisObject visObj : visObjs){
	// 				objs.add(new VisChain(xform, visObj));
	// 			}
	// 			xform = LinAlg.copy(xform);
	// 		} else {
	// 			System.out.println(op);
	// 			assert(false);
	// 		}
	// 	}
	// 	return objs;
	// }
}
