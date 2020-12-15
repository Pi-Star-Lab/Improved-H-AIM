package aim4.map.intersectionboard;

import aim4.im.Intersection;
import aim4.map.lane.Lane;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DifferingLanesPerRoadBoard extends IntersectionBoard {

    /**
     * map from laneId to its corresponding point on board
     */
    private Map<Integer, Point2D> laneInSet, laneOutSet;

    /**
     * Constructor.
     * @param inter Intersection for which the intersection of lanes needs to be checked
     */
    public DifferingLanesPerRoadBoard(Intersection inter) {
        laneInSet = new HashMap<Integer, Point2D>();
        laneOutSet = new HashMap<Integer, Point2D>();

        //this doesn't take lane turn/action restrictions into account. It is purely the layout of the intersection as the simulator is concerned.
        List<Lane> entryLanes = inter.getEntryLanes();
        List<Lane> exitLanes = inter.getExitLanes();
        for (Lane lane : entryLanes) {
            Point2D pt = inter.getEntryPoint(lane);
            if (pt != null && pt.getX() >= 0 && pt.getY() >= 0) {
                laneInSet.put(lane.getId(), new Point2D.Double(pt.getX(), pt.getY()));
            }
        }

        for (Lane lane : exitLanes) {
            Point2D pt = inter.getExitPoint(lane);
            if (pt != null && pt.getX() >= 0 && pt.getY() >= 0) {
                laneOutSet.put(lane.getId(), pt);
            }
        }
    }

    @Override
    public boolean intersects(int firstLaneIn, int firstLaneOut,
            int secondLaneIn, int secondLaneOut) {

        // TODO Auto-generated method stub
        Point2D firstLaneInPoint = laneInSet.get(firstLaneIn);
        Point2D firstLaneOutPoint = laneOutSet.get(firstLaneOut);
        Point2D secondLaneInPoint = laneInSet.get(secondLaneIn);
        Point2D secondLaneOutPoint = laneOutSet.get(secondLaneOut);

        if (secondLaneInPoint == null) {
            System.out.print("null pointer");
        }

        // check whether they intersect
        return Line2D.linesIntersect(firstLaneInPoint.getX(),
                firstLaneInPoint.getY(),
                firstLaneOutPoint.getX(),
                firstLaneOutPoint.getY(),
                secondLaneInPoint.getX(),
                secondLaneInPoint.getY(),
                secondLaneOutPoint.getX(),
                secondLaneOutPoint.getY());
    }
}
