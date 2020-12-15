package aim4.im.v2i.RequestHandler;

import aim4.config.Constants.TurnDirection;
import aim4.config.SimConfig;
import aim4.config.SimConfig.VEHICLE_TYPE;
import aim4.config.TrafficSignal;
import aim4.config.ringbarrier.RBSegmentReadOnlyNoLockingView;
import aim4.config.ringbarrier.RingAndBarrier;
import aim4.im.IntersectionManager;
import aim4.map.Road;
import aim4.map.lane.Lane;
import aim4.util.Util;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 *
 * @author Aaron Parks-Young
 */
/**
 * Fully actuated traffic signal.
 */
public class FullyActuatedSignalController extends AbstractSignalControllerWithLaneID {

    private RingAndBarrier rb;
    private Lane myLane;
    private IntersectionManager im;
    private ApproxNPhasesTrafficSignalRequestHandler requestHandler;

    public FullyActuatedSignalController(RingAndBarrier rb, Lane myLane, IntersectionManager im, ApproxNPhasesTrafficSignalRequestHandler requestHandler) {
        super(myLane.getId());
        this.rb = rb;
        this.myLane = myLane;
        this.im = im;
        this.rb.registerIM(this.im);
        this.requestHandler = requestHandler;
    }

    @Override
    public TrafficSignal getSignal(double time) {
        return getSignal(time, myLane.getLaneIM().validActionsFromLane(im, VEHICLE_TYPE.HUMAN));
    }

    //extends getSignalFromSegmentRange, but allows for specification of turn directions that are desired. This means that a request can ask that at least a particular set of actions is supported by the signal at the current point in time. This is not applicable to looking into the future without changing the RingAndBarrier method to look into the future (which is currently only designed for a single vehicle wanting to go a single direction).
    public synchronized TrafficSignal getSignal(double time, Set<TurnDirection> tds) {
        if (Util.isDoubleEqual(time, requestHandler.getCurrentTime(), SimConfig.TIME_STEP)) {
            return rb.getSignalForLaneWithCurrentTime(time, myLane.getContainingRoad(), tds);
        } else {
            Set<TrafficSignal> results = rb.getSignalForLaneInFuture(requestHandler.getCurrentTime(), time, myLane.getContainingRoad(), tds);
            if (!results.isEmpty()) {
                if (results.size() == 1) {
                    return results.iterator().next();
                } else if (results.size() > 1) {
                    //this is for a single ring, so if there are more than 2 elements, one is not green and so the definite signal is unknown
                    return (results.contains(TrafficSignal.GREEN) ? TrafficSignal.UNKNOWN_CONTAINING_GREEN : TrafficSignal.UNKNOWN);
                }
            }
            return TrafficSignal.UNKNOWN;
        }
    }

    //extends getSignalFromSegmentRange, but allows for specification of turn directions that are desired. This means that a request can ask that at least a particular set of actions is supported by the signal at the current point in time. This is not applicable to looking into the future without changing the RingAndBarrier method to look into the future (which is currently only designed for a single vehicle wanting to go a single direction).
    public synchronized TrafficSignal getSignalFromSegmentRange(Road containingRoad, Set<TurnDirection> tds, Set<RBSegmentReadOnlyNoLockingView> segmentRangeSets) {
        boolean goodInput = false;
        if (!segmentRangeSets.isEmpty()) {
            if (segmentRangeSets.size() == 1) {
                return segmentRangeSets.iterator().next().getColor();
            } else if (segmentRangeSets.size() > 1) {
                for (RBSegmentReadOnlyNoLockingView view : segmentRangeSets) {
                    if (!Collections.disjoint(view.getTurnDirectionsForPhaseSegment(), tds) && view.getRoad() == containingRoad && !view.isBarrier()) {
                        if (view.getColor() == TrafficSignal.GREEN) {
                            //this is for a single ring, so if there are more than 2 elements, one is not green and so the definite signal is unknown
                            return TrafficSignal.UNKNOWN_CONTAINING_GREEN;
                        }
                        goodInput = true;
                    }
                }
            }
        }
        if (!goodInput) {
            throw new RuntimeException("No segment found (excluding barriers) which handles road and turn direction provided.");
        }
        return TrafficSignal.UNKNOWN;
    }

    public synchronized List<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> getPhaseSegmentsAtTimePotentiallyLockingSegmentData(double time) {
        if (Util.isDoubleEqual(time, requestHandler.getCurrentTime(), SimConfig.TIME_STEP)) {
            return rb.getCurrentSegmentViews();
        } else {
            return rb.getSegmentsByRangeAtTimeAsList(time, requestHandler.getCurrentTime());
        }
    }

    public TrafficSignal getSignalForGUI() {
        return rb.getCurrentSignalForGUI(myLane.getContainingRoad(), myLane.getLaneIM().validActionsFromLane(im, VEHICLE_TYPE.HUMAN));
    }

    @Override
    public boolean getSignalPermissiveness(double time) {
        return false;
    }

    @Override
    public void setOffset(double time) {
        throw new UnsupportedOperationException("Not supported for FullyActuatedSignalController.");
    }

    public void logActuationOnLane(double currentTime, double actuationTime, double exitTime, SimConfig.VEHICLE_TYPE vType) {
        rb.logActuationOnLane(myLane, currentTime, actuationTime, exitTime, vType);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int getLaneID() {
        return myLane.getId();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Lane getLane() {
        return myLane;
    }
}
