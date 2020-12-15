package aim4.map.intersectionboard;

import expr.trb.DesignatedLanesExpr;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.Map;

/**
 * For example, lane 1 -> lane 1, lane 9 -> lane 3, corresponding to (5, 0) ->
 * (5, 7), (7, 4) -> (3, 0). Intersect!
 *
 * @author menie
 */
public class StandardIntersectionBoard extends IntersectionBoard {

    /**
     * Constructor.
     */
    public StandardIntersectionBoard() {

        laneInSet = new HashMap<Integer, Point2D>();
        laneOutSet = new HashMap<Integer, Point2D>();
        int lnCount = DesignatedLanesExpr.NUMBER_OF_LANES;
        int limit = lnCount * 2 + 1;

        // for lane in
        int y = lnCount + 1;
        for (int i = 0; i < lnCount; i++) {
            //            laneInSet.put(0, new Point2D.Double(4, 0));
            //            laneInSet.put(1, new Point2D.Double(5, 0));
            //            laneInSet.put(2, new Point2D.Double(6, 0));
            laneInSet.put(i, new Point2D.Double(y + i, 0));
        }

        y = lnCount;
        int iLane = lnCount;
        for (int i = 0; i < lnCount; i++) {
//            laneInSet.put(3, new Point2D.Double(3, 7));
//            laneInSet.put(4, new Point2D.Double(2, 7));
//            laneInSet.put(5, new Point2D.Double(1, 7));

            laneInSet.put(iLane + i, new Point2D.Double(y - i, limit));
        }

        y = lnCount;
        iLane = lnCount * 2;
        for (int i = 0; i < lnCount; i++) {
//            laneInSet.put(6, new Point2D.Double(0, 3));
//            laneInSet.put(7, new Point2D.Double(0, 2));
//            laneInSet.put(8, new Point2D.Double(0, 1));

            laneInSet.put(iLane + i, new Point2D.Double(0, y - i));
        }

        y = lnCount + 1;
        iLane = lnCount * 3;
        for (int i = 0; i < lnCount; i++) {
//            laneInSet.put(9, new Point2D.Double(7, 4));
//            laneInSet.put(10, new Point2D.Double(7, 5));
//            laneInSet.put(11, new Point2D.Double(7, 6));

            laneInSet.put(iLane + i, new Point2D.Double(limit, y + i));
        }

        // for lane out
        y = lnCount + 1;
        for (int i = 0; i < lnCount; i++) {
//            laneOutSet.put(0, new Point2D.Double(4, 7));
//            laneOutSet.put(1, new Point2D.Double(5, 7));
//            laneOutSet.put(2, new Point2D.Double(6, 7));

            laneOutSet.put(i, new Point2D.Double(y + i, limit));
        }

        y = lnCount;
        iLane = lnCount;
        for (int i = 0; i < lnCount; i++) {
//            laneOutSet.put(3, new Point2D.Double(3, 0));
//            laneOutSet.put(4, new Point2D.Double(2, 0));
//            laneOutSet.put(5, new Point2D.Double(1, 0));

            laneOutSet.put(iLane + i, new Point2D.Double(y - i, 0));
        }

        y = lnCount;
        iLane = lnCount * 2;
        for (int i = 0; i < lnCount; i++) {
//            laneOutSet.put(6, new Point2D.Double(7, 3));
//            laneOutSet.put(7, new Point2D.Double(7, 2));
//            laneOutSet.put(8, new Point2D.Double(7, 1));

            laneOutSet.put(iLane + i, new Point2D.Double(limit, y - i));
        }

        y = lnCount + 1;
        iLane = lnCount * 3;
        for (int i = 0; i < lnCount; i++) {
//            laneOutSet.put(9, new Point2D.Double(0, 4));
//            laneOutSet.put(10, new Point2D.Double(0, 5));
//            laneOutSet.put(11, new Point2D.Double(0, 6));

            laneOutSet.put(iLane + i, new Point2D.Double(0, y + i));
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
