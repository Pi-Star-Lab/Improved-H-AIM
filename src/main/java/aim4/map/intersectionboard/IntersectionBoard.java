package aim4.map.intersectionboard;

import java.awt.geom.Point2D;
import java.util.Map;

/**
 * Make each entrance of the lane as a point on the board, so we can know
 * whether two path would intersect.
 *
 *
 *
 * @author menie
 *
 */
public abstract class IntersectionBoard {

    /**
     * map from laneId to its corresponding point on board
     */
    protected Map<Integer, Point2D> laneInSet, laneOutSet;

    /**
     * whether two lanes intersect
     *
     * @param firstLaneIn
     * @param firstLaneOut
     * @param secondLaneIn
     * @param secondlaneOut
     * @return
     */
    public abstract boolean intersects(int firstLaneIn, int firstLaneOut, int secondLaneIn, int secondlaneOut);
}
