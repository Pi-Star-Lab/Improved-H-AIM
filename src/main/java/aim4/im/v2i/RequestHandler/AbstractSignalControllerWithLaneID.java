package aim4.im.v2i.RequestHandler;

import aim4.map.lane.Lane;

/**
 * The abstract class for signal controllers that have a lane assigned to them.
 */
public abstract class AbstractSignalControllerWithLaneID implements ApproxNPhasesTrafficSignalRequestHandler.SignalController {

    protected int laneId;

    AbstractSignalControllerWithLaneID(int laneId) {
        this.laneId = laneId;
    }

    /**
     *
     * @return the laneID for the lane which this controller supervises
     */
    abstract public int getLaneID();

    /**
     *
     * @return the lane which this controller supervises
     */
    abstract public Lane getLane();

}
