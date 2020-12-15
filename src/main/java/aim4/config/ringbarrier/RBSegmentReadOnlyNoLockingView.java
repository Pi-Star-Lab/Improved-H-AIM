package aim4.config.ringbarrier;

import aim4.config.Constants.TurnDirection;
import aim4.config.TrafficSignal;
import aim4.im.IntersectionManager;
import aim4.map.Road;
import aim4.map.lane.Lane;
import java.util.Set;

public class RBSegmentReadOnlyNoLockingView {

    private final RBPhaseSegment segment;

    /**
     * A view of the segment class which offers no guarantees about the results
     * of a call being equal on subsequent calls.
     *
     * @param segment
     */
    RBSegmentReadOnlyNoLockingView(RBPhaseSegment segment) {
        this.segment = segment;
        if (this.segment == null) {
            throw new IllegalArgumentException("Segment may not be null for RBSegmentReadOnlyNoLockingView.");
        }
    }

    public TrafficSignal getColor() {
        return segment.getColor();
    }

    public Road getRoad() {
        return segment.getRoad();
    }

    /**
     * returns the gap extension time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls.
     *
     * @return the gap extension time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls.
     */
    public double getGapTime() {
        return segment.getGapTime();
    }

    public double getMinTime() {
        return segment.getMinTime();
    }

    /**
     * returns the max allowed time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls.
     *
     * @return the max allowed time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls.
     */
    public double getMaxTime() {
        return segment.getMaxTime();
    }

    public boolean isBarrier() {
        return segment.isBarrier();
    }

    public Set<Lane> getLanesAffectedByPhaseForContainingRoad() {
        return segment.getLanesAffectedByPhaseForContainingRoad();
    }

    public boolean areGapExtensionsAllowed() {
        return segment.areGapExtensionsAllowed();
    }

    public IntersectionManager getAssociatedIntersectionManager() {
        return segment.getAssociatedIntersectionManager();
    }

    public RBSegmentReadOnlyNoLockingView getRBSegmentReadOnlyNoLockingViewForNextPhaseSegment() {
        if (segment.getNextPhaseSegment() == null) {
            return null;
        }
        return new RBSegmentReadOnlyNoLockingView(segment.getNextPhaseSegment());
    }

    public RBSegmentReadOnlyNoLockingView getRBSegmentReadOnlyNoLockingViewForPreviousPhaseSegment() {
        if (segment.getPreviousPhaseSegment() == null) {
            return null;
        }
        return new RBSegmentReadOnlyNoLockingView(segment.getPreviousPhaseSegment());
    }

    public long getSegmentId() {
        return segment.getSegmentId();
    }

    public RBSegmentReadOnlyNoLockingView getRBSegmentReadOnlyNoLockingViewForSegmentRepresentingPhase() {
        if (segment.getSegmentRepresentingPhase() == null) {
            return null;
        }
        return new RBSegmentReadOnlyNoLockingView(segment.getSegmentRepresentingPhase());
    }

    public Set<TurnDirection> getTurnDirectionsForPhaseSegment() {
        return segment.getTurnDirectionsForPhaseSegment();
    }

    public boolean isRedOrYellowSegment() {
        return segment.isRedOrYellowSegment();
    }

    public boolean equals(RBSegmentReadOnlyNoLockingView other) {
        return segment == other.segment;
    }

    public boolean getHoldForOtherPhase() {
        return segment.getHoldForOtherPhase();
    }

    public boolean getOtherPhasesAreHolding() {
        return segment.getOtherSegmentsMightHold();
    }
}
