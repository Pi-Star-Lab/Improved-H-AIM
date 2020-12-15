package aim4.config.ringbarrier;

import aim4.config.Constants;
import aim4.config.Resources;
import aim4.config.SimConfig;
import aim4.config.TrafficSignal;
import aim4.im.IntersectionManager;
import aim4.map.Road;
import aim4.map.lane.Lane;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Set;

public class RBPhaseSegment {

    private static final double TIME_WINDOW_FOR_APPROACH_VOLUME_TRACKING_IN_SECONDS = 3600;

    private Road road;
    private double gapTime, minTime, maxTime;
    //this doesn't work the same way as a lane....if there are no segments in this, there are no segments in this
    private EnumSet<Constants.TurnDirection> turnDirectionsForPhaseSegment;
    protected RBPhaseSegment nextPhaseSegment, previousPhaseSegment;
    private TrafficSignal color;
    private boolean legacyIsPlaceholder;
    //Long.MIN_VALUE isn't a valid ID
    private static long nextId = Long.MIN_VALUE + 1;
    private long id;
    private IntersectionManager im;
    private Set<Lane> lanesAffectedByPhaseSegment;

    private RBPhaseSegment segmentRepresentingPhase;
    private boolean isLocked;
    private boolean useAdjustableSegmentTiming;

    private boolean holdForOtherSegment;
    private boolean otherSegmentsAreHolding;
    private Set<RBPhaseSegment> segmentsHolding;
    private RBPhaseSegment segmentToHoldOn;
    private HistoricalRBSegmentInformation activeTimingInfo;

    //note: setting crossTurn, flowTurn, and through all to true for a barrier has the purpose of forcing all lights red/yellow
    public RBPhaseSegment(Road road, double time, TrafficSignal color, boolean crossTurn, boolean through, boolean flowTurn, boolean holdForOtherPhase, boolean otherPhasesAreHolding, boolean useAdjustableSegmentTiming) {
        this(road, 0, time, time, color, crossTurn, through, flowTurn, holdForOtherPhase, otherPhasesAreHolding, useAdjustableSegmentTiming);
    }

    //note: setting crossTurn, flowTurn, and through all to true for a barrier has the purpose of forcing all lights red/yellow
    public RBPhaseSegment(Road road, double time, TrafficSignal color, boolean crossTurn, boolean through, boolean flowTurn, boolean holdForOtherPhase, boolean otherPhasesAreHolding) {
        this(road, 0, time, time, color, crossTurn, through, flowTurn, holdForOtherPhase, otherPhasesAreHolding, SimConfig.USE_ADAPTIVE_TIMING);
    }

    public RBPhaseSegment(RBPhaseSegment phaseToCopy) {
        if (nextId > 0 && nextId + 1 < 0) {
            throw new RuntimeException("The number of RBPhaseSegments that was created exceeded the maximum allowed number. This caused an overflow in IDs. As IDs are currently implemented as a simple long, this is not allowed in order to prevent identical IDs from being assigned to multiple RBPhaseSegments. Note, copying an RBPhaseSegment takes a new ID as well.");
        }
        this.id = nextId++;
        road = phaseToCopy.road;
        gapTime = phaseToCopy.gapTime;
        minTime = phaseToCopy.minTime;
        maxTime = phaseToCopy.maxTime;
        turnDirectionsForPhaseSegment = EnumSet.copyOf(phaseToCopy.turnDirectionsForPhaseSegment);
        nextPhaseSegment = phaseToCopy.nextPhaseSegment;
        previousPhaseSegment = phaseToCopy.previousPhaseSegment;
        color = phaseToCopy.color;
        legacyIsPlaceholder = phaseToCopy.legacyIsPlaceholder;

        im = phaseToCopy.im;
        if (phaseToCopy.lanesAffectedByPhaseSegment != null) {
            lanesAffectedByPhaseSegment = new HashSet<Lane>(phaseToCopy.lanesAffectedByPhaseSegment);
        } else {
            lanesAffectedByPhaseSegment = null;
        }

        holdForOtherSegment = phaseToCopy.holdForOtherSegment;
        otherSegmentsAreHolding = phaseToCopy.otherSegmentsAreHolding;

        segmentRepresentingPhase = phaseToCopy.segmentRepresentingPhase;
        isLocked = phaseToCopy.isLocked;
        useAdjustableSegmentTiming = phaseToCopy.useAdjustableSegmentTiming;
        segmentsHolding = phaseToCopy.segmentsHolding;
        segmentToHoldOn = phaseToCopy.segmentToHoldOn;
    }

    public RBPhaseSegment(Road road, double gapTime, double minTime, double maxTime, TrafficSignal color, boolean crossTurn, boolean through, boolean flowTurn, boolean holdForOtherPhase, boolean otherPhasesAreHolding) {
        this(road, gapTime, minTime, maxTime, color, crossTurn, through, flowTurn, holdForOtherPhase, otherPhasesAreHolding, SimConfig.USE_ADAPTIVE_TIMING);
    }

    public RBPhaseSegment(Road road, double gapTime, double minTime, double maxTime, TrafficSignal color, boolean crossTurn, boolean through, boolean flowTurn, boolean holdForOtherPhase, boolean otherPhasesAreHolding, boolean useAdjustableSegmentTiming) {
        if (nextId > 0 && nextId + 1 < 0) {
            throw new RuntimeException("The number of RBPhaseSegments that was created exceeded the maximum allowed number. This caused an overflow in IDs. As IDs are currently implemented as a simple long, this is not allowed inorder to prevent identical IDs from being assigned to multiple RBPhaseSegments. Note, copying an RBPhaseSegment takes a new ID as well.");
        }
        this.id = nextId++;
        this.road = road;
        this.gapTime = gapTime;

        this.minTime = minTime;
        this.maxTime = maxTime;
        if (maxTime < minTime) {
            throw new IllegalArgumentException("The maximum time for a phase segment cannot be less than the minimum time. Max: " + maxTime + " Min: " + minTime);
        } else if (minTime < 0) {
            throw new IllegalArgumentException("The minimum time for a phase segment cannot be less than 0. Min: " + minTime);
        }
        this.nextPhaseSegment = null;
        this.previousPhaseSegment = null;
        this.color = color;

        legacyIsPlaceholder = false;

        turnDirectionsForPhaseSegment = EnumSet.noneOf(Constants.TurnDirection.class);
        if (crossTurn) {
            turnDirectionsForPhaseSegment.add(Constants.CROSS_TURN_DIRECTION);
        }
        if (through) {
            turnDirectionsForPhaseSegment.add(Constants.TurnDirection.STRAIGHT);
        }
        if (flowTurn) {
            turnDirectionsForPhaseSegment.add(Constants.WITH_TRAFFIC_TURN_DIRECTION);
        }
        this.holdForOtherSegment = holdForOtherPhase;
        this.otherSegmentsAreHolding = otherPhasesAreHolding;
        if (this.holdForOtherSegment && this.otherSegmentsAreHolding) {
            throw new IllegalArgumentException("A phase can't both wait on another phase and be waited on, currently. Check your special characters in your signal file.");
        }

        im = null;
        lanesAffectedByPhaseSegment = null;

        segmentRepresentingPhase = null;
        isLocked = false;
        this.useAdjustableSegmentTiming = useAdjustableSegmentTiming;
        segmentsHolding = new HashSet<RBPhaseSegment>();
        segmentToHoldOn = null;
    }

    public void setNextPhaseSegment(RBPhaseSegment nextPhase) {
        this.nextPhaseSegment = nextPhase;
    }

    public void setPreviousPhaseSegment(RBPhaseSegment previousPhase) {
        this.previousPhaseSegment = previousPhase;
    }

    public boolean setAssociatedIntersectionManager(IntersectionManager im) {
        if (this.im == null && im != null) {
            this.im = im;
            lanesAffectedByPhaseSegment = new HashSet<Lane>();
            for (Lane lane : road.getLanes()) {
                if (lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im) != null && (lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im).equals(Collections.EMPTY_SET) || !Collections.disjoint(turnDirectionsForPhaseSegment, lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im)))) {
                    lanesAffectedByPhaseSegment.add(lane);
                }
            }

            return true;
        } else {
            return false;
        }
    }

    public IntersectionManager getAssociatedIntersectionManager() {
        return im;
    }

    public boolean getHoldForOtherPhase() {
        return holdForOtherSegment;
    }

    public boolean getOtherSegmentsMightHold() {
        return otherSegmentsAreHolding;
    }

    /**
     * This is used to determine the number of vehicles proceeding through the
     * intersection which are taking a specific turn action. This count is used
     * to determine how to adapt signal timing on the fly, as the count of all
     * vehicle types taking a particular action is of interest.
     *
     * @return Returns the lanes which have a TurnDirection mapped which is
     * controlled by this phase segment. This includes ANY vehicle's turn
     * direction mapping.
     */
    public Set<Lane> getLanesAffectedByPhaseForContainingRoad() {
        if (lanesAffectedByPhaseSegment != null) {
            return Collections.unmodifiableSet(lanesAffectedByPhaseSegment);
        } else {
            return null;
        }
    }

    /**
     * Sets the segment representing the phase. For example, if there is a phase
     * whose segments go GREEN->YELLOW->RED, the representative segment for all
     * three of those should be the GREEN segment.
     *
     * @param seg
     * @return
     */
    public boolean setSegmentRepresentingPhase(RBPhaseSegment seg) {
        if (seg.getColor() != TrafficSignal.GREEN) {
            throw new RuntimeException("Representative phase segments for phases are expected to be for green segments, the provided phase's color was " + seg.getColor().name());
        }

        if (segmentRepresentingPhase == null) {
            segmentRepresentingPhase = seg;
            return true;
        }

        return false;
    }

    public RBPhaseSegment getSegmentRepresentingPhase() {
        return segmentRepresentingPhase;
    }

    /**
     *
     * @return the id from the id pool for segments (ie, all longs except for
     * Long.MIN_VALUE)
     */
    public long getSegmentId() {
        return id;
    }

    public Road getRoad() {
        return road;
    }

    public boolean isBarrier() {
        return false;
    }

    /**
     * returns the gap extension time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls. Use
     * getGapTimeAndLock() to lock the value, but be careful not to interrupt
     * normal ring and barrier behavior in doing so.
     *
     * @return the gap extension time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls.
     */
    public double getGapTime() {
        return gapTime;
    }

    public boolean areGapExtensionsAllowed() {
        return gapTime > 0;
    }

    public double getMinTime() {
        if (segmentToHoldOn == null && holdForOtherSegment) {
            throw new RuntimeException("Error, should be waiting on other segment but no segment was set.");
        } else if (segmentToHoldOn == null) {
            return minTime;
        } else {
            return Math.max(minTime, segmentToHoldOn.getMinTime());
        }
    }

    public double getMinTimeAssumingNoSegmentToHoldOnTo() {
        return minTime;
    }

    /**
     * returns the max allowed time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls. Use
     * getMaxTimeAndLock() to lock the value, but be careful not to interrupt
     * normal ring and barrier behavior in doing so.
     *
     * @return the max allowed time currently, though this function offers no
     * guarantee that results will be the same on subsequent calls.
     */
    public double getMaxTime() {
        if (segmentToHoldOn == null && holdForOtherSegment) {
            throw new RuntimeException("Error, should be waiting on other segment but no segment was set.");
        } else if (segmentToHoldOn == null) {
            return maxTime;
        } else {
            return Math.max(maxTime, segmentToHoldOn.getMaxTime());
        }
    }

    public boolean isRedOrYellowSegment() {
        return color == TrafficSignal.RED || color == TrafficSignal.YELLOW;
    }

    /**
     * Sets the lock on elements and retrieves the determined max time after the
     * lock has been initialized
     *
     * @return
     */
    public double getMaxTimeAndLock() {
        lockIfPossible();
        return maxTime;
    }

    /**
     * Sets the lock on elements and retrieves the determined gap extension time
     * after the lock has been initialized
     *
     * @return
     */
    public double getGapTimeAndLock() {
        lockIfPossible();
        return gapTime;
    }

    private void lockIfPossible() {
        if (!isLocked) {
            lockForSegmentAdjustments();
        }
    }

    public Set<Constants.TurnDirection> getTurnDirectionsForPhaseSegment() {
        return Collections.unmodifiableSet(turnDirectionsForPhaseSegment);
    }

    public RBPhaseSegment getNextPhaseSegment() {
        return nextPhaseSegment;
    }

    public RBPhaseSegment getPreviousPhaseSegment() {
        return previousPhaseSegment;
    }

    public TrafficSignal getColor() {
        return color;
    }

    public boolean isLegacyIsPlaceholder() {
        return legacyIsPlaceholder;
    }

    public void setLegacyIsPlaceholder(boolean legacyIsPlaceholder) {
        this.legacyIsPlaceholder = legacyIsPlaceholder;
    }

    public boolean isLocked() {
        return isLocked;
    }

    public void lockForSegmentAdjustments() {
        if (isLocked) {
            throw new RuntimeException("Segment cannot be locked when it is already locked. This could be an indication that a vehicle attempted to make a reservation so far in advance that a future segment lookup looked up all the way back to the current segment. Or changes were recently made that caused this.");
        }
        Set<Lane> lanes = getLanesAffectedByPhaseForContainingRoad();
        if (useAdjustableSegmentTiming && lanes != null) {
            if (im != null && !lanes.isEmpty() && !isRedOrYellowSegment()) {
                double vehicleCount = 0;
                Double allowedAheadTime = null; //this will nullpointerexception if things go wonky, which is good so we'll quickly catch it
                for (Lane lane : lanes) {
                    //todo, might want to put this under the same guard as getMaxTimeBasedOnArrivalDataForAllPhasesInRingWithoutLocking?
                    vehicleCount += lane.getLaneIM().getNumberOfArrivalsOnLaneForTurnDirections(im, turnDirectionsForPhaseSegment);
                    if (allowedAheadTime != null) {
                        allowedAheadTime = Math.min(im.getMaxAllowedFutureReservationTimeOnLane(lane), allowedAheadTime);
                    } else {
                        allowedAheadTime = im.getMaxAllowedFutureReservationTimeOnLane(lane);
                    }
                }

                if (road.getRoadSpeedLimit() == null) {
                    throw new RuntimeException("lockForSegmentAdjustments currently cannot handle a road where the speed limits for lanes differ.");
                }

                if (vehicleCount > 0) {
                    maxTime = getMaxTimeBasedOnArrivalDataForAllPhasesInRingWithoutLocking();
                } else {
                    maxTime = MaximumGreenTable.getMaximumGreenTimeInSecondsClosestToProvidedKeys(vehicleCount / lanes.size(), Resources.ringAndBarrier.getAvgCycleLength());
                }
                maxTime = Math.max(maxTime, getMinTime());
                if (segmentToHoldOn == null && holdForOtherSegment) {
                    throw new RuntimeException("Error, should be waiting on other segment but no segment was set.");
                } else if (segmentToHoldOn == null) {
                    //do nothing
                } else {
                    maxTime = Math.max(maxTime, segmentToHoldOn.getMaxTimeAndLock());
                }

                //https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter5.htm we're basically using a gap reduction method, so we set this value at 4.0. We assume all vehicles will approach at speed limit, so we use that.
                gapTime = GapExtensionTable.getMaximumGapExtensionInSecondsClosestToProvidedKeys(3.0, 1.829, road.getRoadSpeedLimit());
                //gapTime = GapExtensionTable.getMaximumGapExtensionInSecondsClosestToProvidedKeys(4.0, allowedAheadTime * road.getRoadSpeedLimit(), road.getRoadSpeedLimit());
//                System.out.println(getRoad().getName() + " " + color.name() + " " + turnDirectionsForPhaseSegment.toString());
//                System.out.println("Maxtime: " + maxTime);
//                System.out.println("Gaptime: " + gapTime);
//                System.out.println("V Count: " + vehicleCount + "/" + lanes.size());
            }
        }
        isLocked = true;
    }

    private double getMaxTimeBasedOnArrivalDataForAllPhasesInRingWithoutLocking() {
        HashSet<Long> seenSegments = new HashSet<Long>();
        ArrayList<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> views = im.getRingAndBarrier().getCurrentSegmentViews();
        ArrayList<RBSegmentReadOnlyNoLockingView> currentViews = new ArrayList<RBSegmentReadOnlyNoLockingView>(views.size());
        ArrayList<Double> maxTimeEstimateForRing = new ArrayList<Double>(views.size());
        ArrayList<Double> vhl = new ArrayList<Double>(views.size());
        double arrivalsOnLanesForCheckingPhase = 0.0;
        double numberOfLanesForCheckingPhase = 0.0;
        int myRow = 0;

        for (LinkedHashSet<RBSegmentReadOnlyNoLockingView> cViewSet : views) {
            currentViews.add(cViewSet.iterator().next());
            maxTimeEstimateForRing.add(0.0);
            vhl.add(0.0);
        }

        for (int i = 0; i < currentViews.size(); ++i) {
            RBSegmentReadOnlyNoLockingView currentView = currentViews.get(i);
            while (!seenSegments.contains(currentView.getSegmentId())) {
                seenSegments.add(currentView.getSegmentId());
                if (currentView.getColor() != TrafficSignal.GREEN) {
                    currentView = currentView.getRBSegmentReadOnlyNoLockingViewForNextPhaseSegment();
                    continue;
                }

                Set<Lane> lanes = currentView.getLanesAffectedByPhaseForContainingRoad();
                double vehicleCount = 0;
                int numLanes = 0;
                for (Lane lane : lanes) {
                    if (lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im).size() > 1 || !lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im).contains(Constants.WITH_TRAFFIC_TURN_DIRECTION)) {
                        vehicleCount += lane.getLaneIM().getNumberOfArrivalsOnLaneForTurnDirections(im, currentView.getTurnDirectionsForPhaseSegment());
                        numLanes++;
                    }
                }

                //vhl.set(i, vhl.get(i) + vehicleCount / lanes.size());
                vhl.set(i, vhl.get(i) + vehicleCount / numLanes);
                double timeToAdd = MaximumGreenTable.getMaximumGreenTimeInSecondsClosestToProvidedKeys(vehicleCount / lanes.size(), Resources.ringAndBarrier.getAvgCycleLength());

                //double timeToAdd = MaximumGreenTable.getMaximumGreenTimeInSecondsClosestToProvidedKeys(arrivalsForIntersection / numberOfActiveLanes, Resources.ringAndBarrier.getAvgCycleLength());
                maxTimeEstimateForRing.set(i, maxTimeEstimateForRing.get(i) + timeToAdd);
                //we're looking at the current segment
                if (currentView.getSegmentId() == id) {
                    Set<Lane> myLanes = currentView.getLanesAffectedByPhaseForContainingRoad();
                    //numberOfLanesForCheckingPhase = myLanes.size();
                    for (Lane lane : myLanes) {
                        
                        if (lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im).size() > 1 || !lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im).contains(Constants.WITH_TRAFFIC_TURN_DIRECTION)) {
                            arrivalsOnLanesForCheckingPhase += lane.getLaneIM().getNumberOfArrivalsOnLaneForTurnDirections(im, currentView.getTurnDirectionsForPhaseSegment());
                            numberOfLanesForCheckingPhase++;
                        }
                    }
                    myRow = i;
                }

                currentView = currentView.getRBSegmentReadOnlyNoLockingViewForNextPhaseSegment();
            }
        }
        double multiplier = (arrivalsOnLanesForCheckingPhase / numberOfLanesForCheckingPhase) / vhl.get(myRow);
        double result = maxTimeEstimateForRing.get(myRow) * multiplier;
        return (Double.isNaN(result) ? 0 : result);
    }

    public void unlockForSegmentAdjustmentsAndClearActiveTimingCache() {
        isLocked = false;
        activeTimingInfo = null;
    }

    public void addSegmentHolding(RBPhaseSegment seg) {
        if (!otherSegmentsAreHolding) {
            throw new RuntimeException("Can't set segment as paired when otherPhasesAreHolding is false.");
        }
        segmentsHolding.add(seg);
    }

    public void setSegmentToHoldOn(RBPhaseSegment seg) {
        if (!holdForOtherSegment) {
            throw new RuntimeException("Can't set segment as paired when otherPhasesAreHolding is false.");
        }
        segmentToHoldOn = seg;
    }

    public Set<RBPhaseSegment> getSetOfSegmentsHolding() {
        return Collections.unmodifiableSet(segmentsHolding);
    }

    public void setActiveTimingInfo(HistoricalRBSegmentInformation info, double currentTime) {
        activeTimingInfo = info;
        if (holdForOtherSegment && segmentToHoldOn.activeTimingInfo != null) {
            segmentToHoldOn.updateMeOnTiming(this, currentTime);
        }
    }

    public boolean requestThatTimingInfoUpdateToTime(double time, double currentTime) {
        if (activeTimingInfo != null) {
            double currentExpectedEndTime = activeTimingInfo.getSimTimeWhenExpectedEndTimeExpires();
            activeTimingInfo.setSimTimeWhenUpdated(currentTime);
            activeTimingInfo.setSimTimeWhenExpectedEndTimeExpires(Math.max(activeTimingInfo.getSimTimeWhenExpectedEndTimeExpires(), Math.min(time, activeTimingInfo.getSimTimeWhenMaxTimeExpires())));
            if (currentExpectedEndTime != activeTimingInfo.getSimTimeWhenExpectedEndTimeExpires()) {
                activeTimingInfo.setComparisonAsStaleForOwningRing();
            }
            return true;
        }

        return false;
    }

    private void updateMeOnTiming(RBPhaseSegment segToUpdate, double currentTime) {
        segToUpdate.requestThatTimingInfoUpdateToTime(activeTimingInfo.getSimTimeWhenExpectedEndTimeExpires(), currentTime);
    }
}
