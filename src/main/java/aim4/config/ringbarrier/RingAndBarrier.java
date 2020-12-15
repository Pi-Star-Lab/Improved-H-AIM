package aim4.config.ringbarrier;

import aim4.config.Constants;
import aim4.config.Constants.TurnDirection;
import aim4.config.Resources;
import aim4.config.SimConfig;
import aim4.config.TrafficSignal;
import aim4.im.IntersectionManager;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler;
import aim4.map.Road;
import aim4.map.lane.Lane;
import aim4.util.Util;
import aim4.vehicle.VehicleSimView;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;

/**
 * Ring and barrier object. Expects time steps to be consistent and constant.
 */
public class RingAndBarrier {

    private static final short CYCLE_MOVING_AVERAGE_WINDOW_SIZE = 3; //dropoutQueue only works if this is small

    private boolean fromLegacy;
    private Double LEAGACY_redOverride;
    private double timeForNextChangesCheck;
    private double timeForAdvanceToBarrier;
    private HashMap<Integer, RBRing> rings;
    private ArrayList<Integer> sortedRingIds;
    private int ringsReadyToAdvanceBarrier;
    private ActuationTracker actTrack;
    private PriorityQueue<RBRing> nextUpdate;
    private IntersectionManager im;
    private double movingAverageOfCycleLength;
    private Queue<Double> dropoutQueue; //this only works if CYCLE_MOVING_AVERAGE_WINDOW_SIZE is small
    private short movingAverageStartupCount;
    private double cycleEpoch;

    public RingAndBarrier(boolean fromLegacy, List<RBRing> rings) {
        this.fromLegacy = fromLegacy;
        this.rings = new HashMap<Integer, RBRing>(rings.size());
        sortedRingIds = new ArrayList<Integer>(rings.size());
        for (RBRing ring : rings) {
            if (this.rings.keySet().contains(ring.getRingId())) {
                throw new RuntimeException("RingAndBarrier object received multiple rings with the same ID.");
            }
            sortedRingIds.add(ring.getRingId());
            this.rings.put(ring.getRingId(), ring);
        }
        Collections.sort(sortedRingIds);

        timeForAdvanceToBarrier = Double.NEGATIVE_INFINITY;
        LEAGACY_redOverride = null;
        ringsReadyToAdvanceBarrier = 0;
        actTrack = new ActuationTracker();

        nextUpdate = null; //handled after IM is registers

        movingAverageOfCycleLength = 0;
        movingAverageStartupCount = 0;
        cycleEpoch = 0;
        dropoutQueue = new LinkedList<Double>();
    }

    private void updateCycleLengthMovingAverage(double lastCycleLength) {
        if (movingAverageStartupCount == CYCLE_MOVING_AVERAGE_WINDOW_SIZE) {
            dropoutQueue.add(lastCycleLength);
            //simple moving average https://en.wikipedia.org/wiki/Moving_average#Simple_moving_average
            movingAverageOfCycleLength += (1.0 / CYCLE_MOVING_AVERAGE_WINDOW_SIZE) * (lastCycleLength - dropoutQueue.poll());
        } else {
            dropoutQueue.add(lastCycleLength);
            //this is a cumulative moving average up to adding the CYCLE_MOVING_AVERAGE_WINDOW_SIZEth element
            //https://en.wikipedia.org/wiki/Moving_average#Cumulative_moving_average
            movingAverageOfCycleLength += (lastCycleLength - movingAverageOfCycleLength) / (movingAverageStartupCount + 1.0);
            ++movingAverageStartupCount;
        }
        //System.out.println("avg: " + movingAverageOfCycleLength);
    }

    public ArrayList<LinkedList<HistoricalRBSegmentInformation>> getCurrentSegmentTimingInfoSortedByRingId() {
        return getSegmentTimingInfoSortedByRingId(false);
    }

    public ArrayList<LinkedList<HistoricalRBSegmentInformation>> getPreviousSegmentTimingInfoSortedByRingId() {
        return getSegmentTimingInfoSortedByRingId(true);
    }

    public void registerIM(IntersectionManager im) {
        if (this.im == null) {
            this.im = im;
        } else if (this.im != im) {
            throw new RuntimeException("IntersectionManager already assigned. Cannot assign a different one.");
        }
    }

    public boolean bindPhaseSegmentsToIM() {
        if (im == null) {
            return false;
        } else {
            nextUpdate = new PriorityQueue<RBRing>();
            for (RBRing ring : rings.values()) {
                ring.bindIMToSegments(im);
            }
            //have to do this after the current segments are set up by the bind im above
            for (RBRing ring : rings.values()) {
                ring.setUpSegmentTimingInformation();
                double ringUpdateTime = ring.getCurrentSegmentExpectedEnd();
                timeForAdvanceToBarrier = Math.max(ringUpdateTime, timeForAdvanceToBarrier);
                nextUpdate.add(ring);
            }
            return true;
        }
    }

    /**
     *
     * @param shouldBePrevious boolean flag, if true select the previous
     * RBSegmentTimingAndColorInformations for each ring, else select the
     * current
     * @return ArrayList of linked lists of HistoricalRBSegmentInformation
     * sorted by their respective ring ID.
     */
    private ArrayList<LinkedList<HistoricalRBSegmentInformation>> getSegmentTimingInfoSortedByRingId(boolean shouldBePrevious) {
        ArrayList<LinkedList<HistoricalRBSegmentInformation>> sortedSegmentInfo = new ArrayList<LinkedList<HistoricalRBSegmentInformation>>(rings.size());
        //since the ids are sorted here, we'll be able to loop through the rings
        for (int id : sortedRingIds) {
            RBRing ring = rings.get(id);
            if (ring == null) {
                throw new RuntimeException("Null ring found in Ring and Barrier object while returning sorted list of rings.");
            } else if (shouldBePrevious) {
                sortedSegmentInfo.add(ring.getAndClearHistoricalTimingInformation());
            } else if (!shouldBePrevious) {
                LinkedList<HistoricalRBSegmentInformation> innerList = new LinkedList<HistoricalRBSegmentInformation>();
                innerList.add(ring.getCurrentSegmentTimingInformationCopy());
                sortedSegmentInfo.add(innerList);
            }
        }
        return sortedSegmentInfo;
    }

    /**
     *
     * @return ArrayList of linked lists sorted by their respective ring ID of
     * sets of ids for the phase segments within each phase.
     */
    public ArrayList<List<Set<Long>>> getSegmentIdsInPhases() {
        ArrayList<List<Set<Long>>> sortedPhaseInfo = new ArrayList<List<Set<Long>>>(rings.size());
        //since the ids are sorted here, we'll be able to loop through the rings
        for (int id : sortedRingIds) {
            RBRing ring = rings.get(id);
            if (ring == null) {
                throw new RuntimeException("Null ring found in Ring and Barrier object while returning sorted list of rings.");
            }
            sortedPhaseInfo.add(ring.getSegmentIdsForPhases());
        }
        return sortedPhaseInfo;
    }

    public Double getLatestEndOfYellowOrRedForRingWithCurrentSegmentHandlingTurnDirectionForRoadAndLockIfNeeded(Road road, Constants.TurnDirection td) {
        Double timeOfNextBarrier = null; //used in case we need to cross a barrier to get the appropriate times.

        for (RBRing ring : rings.values()) {
            //look at the road and direction for the most recent green light, just in case we're in a barrier
            RBPhaseSegment seg = ring.getCurrentPhaseSegment();
            if (seg.getSegmentRepresentingPhase().getRoad() == road && seg.getSegmentRepresentingPhase().getTurnDirectionsForPhaseSegment().contains(td)) {
                //we found the right ring! (assuming only one ring handles this action for this road)
                HistoricalRBSegmentInformation timingInfo = ring.getCurrentSegmentTimingInformationCopy();
                if (timingInfo.getColor() == TrafficSignal.YELLOW) {
                    //we found the right segment!
                    //we're in the segment, so no need to lock it.
                    //return timingInfo.getLatestKnownEndTime();
                    RBPhaseSegment nextSegment = ring.getCurrentPhaseSegment().nextPhaseSegment;
                    timingInfo = new HistoricalRBSegmentInformation(ring, nextSegment.getSegmentId(), timingInfo.getLatestKnownEndTime(), nextSegment.getMinTime(), nextSegment.getMaxTimeAndLock(), seg.getMaxTimeAndLock() + timingInfo.getLatestKnownEndTime(), timingInfo.getLatestKnownEndTime(), ring.areEarlyGapoutsPermitted(), nextSegment.getColor(), nextSegment.areGapExtensionsAllowed());

                    return timingInfo.getLatestKnownEndTime();
                } else if (timingInfo.getColor() == TrafficSignal.GREEN) {
                    if (timeOfNextBarrier == null) {
                        for (RBRing ring_again : rings.values()) {
                            SignalRangeCheckDTO tempDTO = new SignalRangeCheckDTO(im.getCurrentTime(), ring_again.getCurrentSegmentTimingInformationCopy(), ring_again.getCurrentPhaseSegment(), 0, ring_again);
                            getTimeOfNextBarrierAndMutateSrcDtoInPlace(tempDTO, false); //use the latest possible time for each segment to advance to barrier
                            timeOfNextBarrier = (timeOfNextBarrier == null ? tempDTO.time : Math.max(timeOfNextBarrier, tempDTO.time));
                        }
                    }
                    RBPhaseSegment nextSegment = ring.getCurrentPhaseSegment().nextPhaseSegment;
                    //the time the yellow phase begins is math.min(time to barrier, time to next phase)
                    double timeOfEpoch = Math.min(timingInfo.getLatestKnownEndTime(), timeOfNextBarrier);
                    timingInfo = new HistoricalRBSegmentInformation(ring, nextSegment.getSegmentId(), timeOfEpoch, nextSegment.getMinTime(), nextSegment.getMaxTimeAndLock(), seg.getMaxTimeAndLock() + timeOfEpoch, timeOfEpoch, ring.areEarlyGapoutsPermitted(), nextSegment.getColor(), nextSegment.areGapExtensionsAllowed());
                    return timingInfo.getLatestKnownEndTime();
                } else if (td != Constants.WITH_TRAFFIC_TURN_DIRECTION) {
                    //a call for the signal color being red
                    //we found the right segment!
                    //we're in the segment, so no need to lock it.
                    return timingInfo.getLatestKnownEndTime();
                    //throw new RuntimeException("getLatestEndOfYellowOrRedForRingWithCurrentSegmentHandlingTurnDirectionForRoadAndLockIfNeeded was called when the signal was red. Was this an error/running a red light?");
                }
            }
        }

        return null;
    }

    public Set<TrafficSignal> getSignalForLaneInFuture(double currentTime, double time, Road roadContainingLane, Set<TurnDirection> tds) {
        return getSignalForLaneInFuture(currentTime, time, roadContainingLane, tds, new ArrayList<Set<RBSegmentReadOnlyNoLockingView>>(getSegmentsByRangeAtTime(time, currentTime).values()));
    }

    /**
     * This is used for vehicles to look ahead to see if a reservation can be
     * approved. Note, the check to see if the allowed turning movements and the
     * desired turning movements
     *
     * @param currentTime
     * @param time
     * @param roadContainingLane
     * @param tds
     * @param rangeOfSegments
     * @return
     */
    public Set<TrafficSignal> getSignalForLaneInFuture(double currentTime, double time, Road roadContainingLane, Set<TurnDirection> tds, List<Set<RBSegmentReadOnlyNoLockingView>> rangeOfSegments) {
        HashMap<Integer, Set<TrafficSignal>> retMap = new HashMap<Integer, Set<TrafficSignal>>();
        List<Set<RBSegmentReadOnlyNoLockingView>> ringsForSignalAtTimeBroadRange = rangeOfSegments;

        if (tds == null) {
            return EnumSet.of(TrafficSignal.UNKNOWN);
        } else if (tds.size() != 1) {
            throw new UnsupportedOperationException("Multiple turn directions are currently not supported in future lane lookup. To add this functionality, you must handle vehicle requests (which should always be 1 turn direction) differently than when this function is called for the reason you're calling it.");
        }

        Integer correctRingListIndex = null;

        //todo, optimize this with a map or something....
        for (int ring = 0; ring < ringsForSignalAtTimeBroadRange.size(); ++ring) {
            Set<RBSegmentReadOnlyNoLockingView> segmentSet = ringsForSignalAtTimeBroadRange.get(ring);
            if (segmentSet == null || segmentSet.isEmpty()) {
                throw new RuntimeException("Got no future segments for a ring. This should never occur.");
            } else {
                if (!retMap.containsKey(ring)) {
                    retMap.put(ring, EnumSet.noneOf(TrafficSignal.class));
                }
                Iterator<RBSegmentReadOnlyNoLockingView> segmentViews = segmentSet.iterator();

                while (segmentViews.hasNext()) {
                    RBSegmentReadOnlyNoLockingView segmentView = segmentViews.next();
                    //System.out.print("\t" + segmentView.getRoad().getName() + ", " + segmentView.getTurnDirectionsForPhaseSegment() + ", " + segmentView.getColor() + "\n");
                    if (segmentView != null) {
                        if (segmentView.getRoad() == roadContainingLane) {
                            //an empty check isn't needed here so long as an only one turn direction check is permitted (as is done using the exception above in this function that states "Multiple turn directions are currently not supported in future...")
                            if (!Collections.disjoint(segmentView.getTurnDirectionsForPhaseSegment(), tds)) {
                                retMap.get(ring).add(segmentView.getColor());
                                //barriers "handle" all turn directions, so they shouldn't be considered in this check
                                if (!segmentView.isBarrier()) {
                                    if (correctRingListIndex != null && ring != correctRingListIndex) {
                                        throw new RuntimeException("When looking at future signals, multiple rings could be the correct ring for the turn direction provided.");
                                    }
                                    correctRingListIndex = ring;
                                }
                            }
                        } else {
                            retMap.get(ring).add(TrafficSignal.RED);
                        }
                    }
                }
            }
        }

        if (correctRingListIndex != null) {
            return retMap.get(correctRingListIndex);
        } else {
            return EnumSet.of(TrafficSignal.UNKNOWN);
        }
    }

    public ArrayList<LinkedList<Set<Constants.TurnDirection>>> getTurnDirectionsByRingSeparatedByPhaseSortedByRingId() {
        ArrayList<LinkedList<Set<Constants.TurnDirection>>> retList = new ArrayList<LinkedList<Set<Constants.TurnDirection>>>(sortedRingIds.size());
        for (int ringId : sortedRingIds) {
            RBRing ring = rings.get(ringId);
            retList.add(ring.getTurnDirectionsForPhases());
        }
        return retList;
    }

    public ArrayList<LinkedList<Double>> getStreetDirectionsByRingSeparatedByPhaseSortedByRingId() {
        if (im == null) {
            throw new RuntimeException("Intersection manager is null. An intersection manager must be assigned before calling this method.");
        }

        ArrayList<LinkedList<Double>> retList = new ArrayList<LinkedList<Double>>(sortedRingIds.size());
        for (int ringId : sortedRingIds) {
            RBRing ring = rings.get(ringId);
            retList.add(ring.getStreetDirectionsForPhases(im));
        }
        return retList;
    }

    public TrafficSignal getSignalForLaneWithCurrentTime(double currentTime, Road roadContainingLane, Set<TurnDirection> tds) {
        //todo, still a little ineffecient. Should have a list of when to check each ring.
        checkForPhaseTransitionAndAdvance(currentTime);

        //todo, optimize this with a map or something....
        if (tds != null) {
            for (RBRing ring : rings.values()) {
                RBPhaseSegment segment = ring.getCurrentPhaseSegment();
                ring.updateUpdateTimeOfSegmentTimingInformation(currentTime);
                if (segment.getRoad() == roadContainingLane) {
                    if (!Collections.disjoint(segment.getTurnDirectionsForPhaseSegment(), tds) || tds.equals(Collections.EMPTY_SET)) {
                        return segment.getColor();
                    }
                }
            }
        }

        return TrafficSignal.RED; //return red rather than unknown because it is either a currently actively red signal, or it doesn't exist.
    }

    public void refreshSignals(double currentTime) {
        checkForPhaseTransitionAndAdvance(currentTime);
    }

    //function will stay on the side of caution and return false if unknown
    public boolean isPermissiveAtTime(Lane lane, double time, double currentTime) {
        /*HashMap<RBRing, Boolean> retMap = new HashMap<RBRing, Boolean>();
        HashMap<RBRing, Set<RBPhaseSegment>> ringsForSignalAtTimeBroadRange = getSignalRange(time, currentTime);

        RBRing correctRing = null;

        //todo, optimize this with a map or something....
        for (RBRing ring : ringsForSignalAtTimeBroadRange.keySet()) {
            Set<RBPhaseSegment> segmentSet = ringsForSignalAtTimeBroadRange.get(ring);
            if (segmentSet == null || segmentSet.isEmpty()) {
                throw new RuntimeException("Got no future segments for a ring. This should never occur.");
            } else {
                if (!retMap.containsKey(ring)) {
                    retMap.put(ring, true);
                }
                Iterator<RBPhaseSegment> segments = segmentSet.iterator();

                while (segments.hasNext()) {
                    RBPhaseSegment segment = segments.next();

                    if (segment != null) {
                        retMap.put(ring, retMap.get(ring) && segment.getTurnDirectionsForPhaseSegment().contains(Constants.CROSS_TURN_DIRECTION));

                        if (segment.getRoad() == lane.getContainingRoad() && segment.getTurnDirectionsForPhaseSegment().contains(Constants.CROSS_TURN_DIRECTION)) {
                            correctRing = ring;
                        }
                    }
                }
            }
            if (correctRing != null) {
                break;
            }
        }

        if (correctRing != null) {
            return retMap.get(correctRing);
        } else {
            return false;
        }*/
        return false;
    }

    private void checkForPhaseTransitionAndAdvance(double currentTime) {
        PriorityQueue<RBRing> tempQueue = new PriorityQueue<RBRing>();
        HashSet<RBRing> ringsReadyToCrossIntoBarrier = new HashSet<RBRing>();
        double latestTime = Double.NEGATIVE_INFINITY;
        Double allRingsLoopedTime = null;
        HashSet<RBRing> ringsNotTouched = new HashSet<RBRing>();
        ringsNotTouched.addAll(rings.values());
        double previousCycleEpoch = cycleEpoch;

        //check if the next update of the ring was moved due to something like an actuation. If it is, check if it's new update time has already passed. If the update time has passed, whether stale or not, process it. Otherwise, drop it in a new list for readding.
        while (!nextUpdate.isEmpty() && (nextUpdate.peek().comparisonMayBeStale() || currentTime >= nextUpdate.peek().getCurrentSegmentExpectedEnd())) {
            if (nextUpdate.peek().comparisonMayBeStale() && currentTime < nextUpdate.peek().getCurrentSegmentExpectedEnd()) {
                RBRing ring = nextUpdate.poll();
                ring.updateUpdateTimeOfSegmentTimingInformation(currentTime);
                ringsNotTouched.remove(ring);
                ring.resetComparisonMayBeStale();
                tempQueue.add(ring);
            } else {
                RBRing ring = nextUpdate.poll();
                HistoricalRBSegmentInformation rbsi = ring.getCurrentSegmentTimingInformationCopy();
                boolean aboutToLoop = ring.phaseIsLastBarrierPhaseInCycle(rbsi.getId());
                if (!ring.advanceSegmentIfNotCrossingIntoBarrier(currentTime, false, actTrack)) { //ring didn't cross, and we know it's ready to cross by the timing, so it must be stuck at a barrier
                    ringsReadyToCrossIntoBarrier.add(ring);
                    ring.updateUpdateTimeOfSegmentTimingInformation(currentTime);
                    ringsNotTouched.remove(ring);
                    latestTime = Math.max(latestTime, ring.getCurrentSegmentExpectedEnd());
                } else {
                    //here, we've already transitioned to the next segment
                    //ring.updateUpdateTimeOfSegmentTimingInformation(currentTime); //not needed, as the advance segment does this for us
                    ringsNotTouched.remove(ring);
                    nextUpdate.add(ring);
                    if (!aboutToLoop && allRingsLoopedTime != null) {
                        throw new RuntimeException("Some ring didn't loop while others did when phase segments were advanced.");
                    } else if (aboutToLoop) {
                        //this min operation should really be moot, as all rings should transition out of the barrier at the same time.
                        allRingsLoopedTime = (allRingsLoopedTime == null ? rbsi.getEpoch() : Math.min(allRingsLoopedTime, rbsi.getEpoch()));
                    }
                }
            }
        }

        for (RBRing ring : ringsNotTouched) {
            ring.updateUpdateTimeOfSegmentTimingInformation(currentTime);
        }

        //add the smaller queue to the larger
        if (tempQueue.size() > nextUpdate.size()) {
            tempQueue.addAll(nextUpdate);
            nextUpdate = tempQueue;
        } else if (tempQueue.size() > 0) {
            nextUpdate.addAll(tempQueue);
        } else if (ringsReadyToCrossIntoBarrier.size() == rings.size()) { //every ring is ready to cross the barrier
            for (RBRing ring : ringsReadyToCrossIntoBarrier) {
                ring.forceAdvanceSegment(currentTime, false, actTrack, latestTime);
            }
        }

        //make sure the ringsReadyToCrossBarrier get added back to the queue
        nextUpdate.addAll(ringsReadyToCrossIntoBarrier);

        if (allRingsLoopedTime != null) {
            cycleEpoch = currentTime;
            updateCycleLengthMovingAverage(allRingsLoopedTime - previousCycleEpoch);
        }
    }

    //similar to getSignalForLaneWithCurrentTime, but doesn't do any updates of the signal
    public TrafficSignal getCurrentSignalForGUI(Road roadContainingLane, Set<TurnDirection> tds) {
        //todo, optimize this with a map or something....
        if (tds != null) {
            for (RBRing ring : rings.values()) {
                RBPhaseSegment segment = ring.getCurrentPhaseSegment();
                if (segment.getRoad() == roadContainingLane) {
                    if (!Collections.disjoint(segment.getSegmentRepresentingPhase().getTurnDirectionsForPhaseSegment(), tds) || tds.equals(Collections.EMPTY_SET)) {
                        return segment.getColor();
                    }
                }
            }
        }
        return TrafficSignal.RED; //return red rather than unknown because it is either a currently actively red signal, or it doesn't exist.
    }

    public void checkForEarlyGapoutDueToVehiclesNotApproaching(Lane incomingLane) {
        if (!SimConfig.SIM_ALLOWS_EARLY_GAPOUT) {
            return;
        }
        Set<TurnDirection> incomingLaneTds = incomingLane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im);

        for (RBRing ring : rings.values()) {
            if (ring.getCurrentSegmentTimingInformationCopy().isGappedOutEarly()) {
                continue;
            }

            boolean anyVehicleMayArriveInTime = false;
            double expectedEndTime = ring.getCurrentSegmentExpectedEnd();

            //check that we even care...the segment we have is green, the segment allows early gapouts, the lane allows turn directions, the roads for the lane and the segment match, and the incoming lane and current segments have shared turning actions
            if (!ring.getCurrentPhaseSegment().isRedOrYellowSegment() && ring.getCurrentSegmentTimingInformationCopy().isEarlyGapoutAllowed() && incomingLaneTds != null
                    && ring.getCurrentPhaseSegment().getRoad() == incomingLane.getContainingRoad()
                    && (incomingLaneTds == Collections.EMPTY_SET || ring.getCurrentPhaseSegment().getTurnDirectionsForPhaseSegment() == Collections.EMPTY_SET || !Collections.disjoint(incomingLaneTds, ring.getCurrentPhaseSegment().getTurnDirectionsForPhaseSegment()))) {
                for (Lane lane : ring.getCurrentPhaseSegment().getLanesAffectedByPhaseForContainingRoad()) {
                    if (ring.getCurrentSegmentTimingInformationCopy().isGappedOutEarly() || anyVehicleMayArriveInTime) {
                        break;
                    }

                    for (int vin : Resources.laneToVin.get(lane)) {
                        if (ring.getCurrentSegmentTimingInformationCopy().isGappedOutEarly() || anyVehicleMayArriveInTime) {
                            break;
                        }

                        VehicleSimView vehicle = Resources.vinToVehicles.get(vin);

                        if (Util.vehicleCouldPossiblyArriveWithinTime(im, vehicle, expectedEndTime - im.getCurrentTime())) {
                            anyVehicleMayArriveInTime = true;
                            break;
                        }
                    }
                }
            }

            if (!anyVehicleMayArriveInTime) {
                ring.flagEarlyGapout(im.getCurrentTime());
            }
        }
    }

    public void notifySegmentIfNeededThatVehicleHasEnteredToResetEarlyGapoutCheckTimer(double arrivalTime, Lane arrivalLane) {
        Set<TurnDirection> arrivalLaneTds = arrivalLane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im);
        for (RBRing ring : rings.values()) {
            if (!ring.getCurrentPhaseSegment().isRedOrYellowSegment() && ring.getCurrentSegmentTimingInformationCopy().isEarlyGapoutAllowed() && arrivalLaneTds != null
                    && ring.getCurrentPhaseSegment().getRoad() == arrivalLane.getContainingRoad()
                    && (arrivalLaneTds == Collections.EMPTY_SET || ring.getCurrentPhaseSegment().getTurnDirectionsForPhaseSegment() == Collections.EMPTY_SET || !Collections.disjoint(arrivalLaneTds, ring.getCurrentPhaseSegment().getTurnDirectionsForPhaseSegment()))) {
                ring.resetCurrentEarlyGapoutTimerForPhaseHistorical(arrivalTime);
            }
        }
    }

    public ArrayList<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> getCurrentSegmentViews() {
        ArrayList<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> retVals = new ArrayList<LinkedHashSet<RBSegmentReadOnlyNoLockingView>>();
        for (int ri : sortedRingIds) {
            RBRing ring = rings.get(ri);
            LinkedHashSet<RBSegmentReadOnlyNoLockingView> setForReturn = new LinkedHashSet<RBSegmentReadOnlyNoLockingView>();
            setForReturn.add(new RBSegmentReadOnlyNoLockingView(ring.getCurrentPhaseSegment()));
            retVals.add(setForReturn);
        }
        return retVals;
    }

    //todo, again, probably pretty ineffecient
    public void logActuationOnLane(Lane lane, double currentTime, double actuationTime, double exitTime, SimConfig.VEHICLE_TYPE vType) {
        if (!SimConfig.ALLOW_ACTUATION) {
            return;
        }

        if (im == null) {
            throw new RuntimeException("Intersection manager is null. An intersection manager must be assigned before calling this method.");
        }

        //Set<TurnDirection> turnActs = lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im); //use this instead if the vehicle type can't be intuited (in real life) based on whether the sensor actuation and vehicle intersection entrance is meeting a reservation or not. If it lines up with a reservation, it's an AV. If not, it's an HV.
        Set<TurnDirection> turnActs = lane.getLaneIM().validActionsFromLane(im, vType);
        actTrack.logActuation(currentTime, actuationTime, exitTime, lane.getContainingRoad(), turnActs);
        for (RBRing ring : rings.values()) {
            ring.updateUpdateTimeOfSegmentTimingInformation(currentTime);
            if (lane.getContainingRoad() == ring.getCurrentPhaseSegment().getRoad() && turnActs != null && (!Collections.disjoint(ring.getCurrentPhaseSegment().getTurnDirectionsForPhaseSegment(), turnActs) || turnActs.equals(Collections.EMPTY_SET))) {
                ring.logActuation(currentTime, actTrack);
            }
        }
    }

    public int getNumberOfRings() {
        return rings.size();
    }

    /**
     * Generate a signal controller for a road. LEGACY only. This actually is a
     * static initialization of CyclicSignalController, not a construction.
     * NOTE: needs to be verified.
     *
     * @param road the road
     * @return a signal controller for the road
     */
    public ApproxNPhasesTrafficSignalRequestHandler.CyclicSignalController LEGACY_calcCyclicSignalController(Road road) {
        if (!fromLegacy) {
            throw new UnsupportedOperationException("Legacy operation requested for non-legacy construction of RingAndBarrier.");
        }

        RBRing ringForRoad = null;

        Iterator<RBRing> iterator = rings.values().iterator();

        while (iterator.hasNext()) {
            RBRing tempRingRef = iterator.next();
            Road rd = tempRingRef.getFirstRoadInRing();
            if (rd != null && rd == road) {
                ringForRoad = tempRingRef;
                break;
            }
        }

        if (ringForRoad != null) {
            int totalCount = ringForRoad.getPhaseGroupsCount();
            double[] durations = new double[totalCount * 3];
            TrafficSignal[] signals = new TrafficSignal[totalCount * 3];
            boolean[] permissive = new boolean[totalCount * 3];

            int count = 0;
            int barrierCount = 0;
            Iterator<RBPhaseSegment> psIt = ringForRoad.getPhaseSegments().iterator();

            if (ringForRoad.getPhaseSegments().size() % 3 != 0) {
                throw new RuntimeException("When trying to create CyclicSignalController, unexpected number of phase segments found. Should be a multiple of 3, but wasn't.");
            }
            while (psIt.hasNext()) {
                RBPhaseSegment segment = psIt.next();
                if (!segment.isLegacyIsPlaceholder()) {
                    durations[count] = segment.getMinTime();
                    signals[count] = TrafficSignal.GREEN;
                    permissive[count] = segment.getTurnDirectionsForPhaseSegment().contains(Constants.CROSS_TURN_DIRECTION);
                    count++;
                    durations[count] = psIt.next().getMinTime();
                    signals[count] = TrafficSignal.YELLOW;
                    permissive[count] = false;
                    count++;
                    double tempRed = psIt.next().getMinTime();
                    durations[count] = (LEAGACY_redOverride != null ? LEAGACY_redOverride : tempRed);
                    signals[count] = TrafficSignal.RED;
                    permissive[count] = false;
                    count++;
                    barrierCount++;
                } else {
                    double tempGreenPlusRed = segment.getMinTime() + psIt.next().getMinTime();
                    double tempRed = psIt.next().getMinTime();
                    durations[count] += tempGreenPlusRed
                            + (LEAGACY_redOverride != null ? LEAGACY_redOverride : tempRed);
                    signals[count] = TrafficSignal.RED;
                    permissive[count] = false;
                    count++;
                    barrierCount++;
                }
            }

            return new ApproxNPhasesTrafficSignalRequestHandler.CyclicSignalController(
                    durations, signals, permissive);
        } else {
            throw new RuntimeException("Couldn't create CyclicSignalController for road: " + road.getName() + ", no phases for road in file.");
        }
    }

    /**
     * Setting the red signal time by passing parameters in running environment.
     * This helps to find the best policy by changing red phase time. NOTE:
     * needs to be verified.
     *
     * @param redPhaseTime time for red signal
     */
    public void LEGACY_resetRedDurations(double redPhaseTime) {
        if (!fromLegacy) {
            throw new UnsupportedOperationException("Legacy operation requested for non-legacy construction of RingAndBarrier.");
        }
        LEAGACY_redOverride = redPhaseTime;
    }

    public List<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> getSegmentsByRangeAtTimeAsList(double futureTime, double currentTime) {
        HashMap<RBRing, LinkedHashSet<RBSegmentReadOnlyNoLockingView>> results = getSegmentsByRangeAtTime(futureTime, currentTime);
        ArrayList<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> returnVals = new ArrayList<LinkedHashSet<RBSegmentReadOnlyNoLockingView>>();
        for (int ri : sortedRingIds) {
            RBRing ring = rings.get(ri);
            if (results.keySet().contains(ring) && !results.get(ring).isEmpty()) {
                returnVals.add(results.get(ring));
            }
        }
        return returnVals;
    }

    /**
     * This function looks ahead to see what segments may occur at the future
     * time provided. This is gives a broad window, using the earliest and
     * latest segment end times possible, not considering state such as
     * currently known actuations.
     *
     * @param futureTime
     * @param currentTime
     * @return
     */
    public HashMap<RBRing, LinkedHashSet<RBSegmentReadOnlyNoLockingView>> getSegmentsByRangeAtTime(double futureTime, double currentTime) {
        int r = rings.size();
        HashMap<RBRing, SignalRangeCheckDTO> earlyTimes = new HashMap<RBRing, SignalRangeCheckDTO>();
        HashMap<RBRing, SignalRangeCheckDTO> lateTimes = new HashMap<RBRing, SignalRangeCheckDTO>();
        HashMap<RBRing, LinkedHashSet<RBSegmentReadOnlyNoLockingView>> results = new HashMap<RBRing, LinkedHashSet<RBSegmentReadOnlyNoLockingView>>();

        for (int ri : sortedRingIds) {
            RBRing ring = rings.get(ri);
            earlyTimes.put(ring, new SignalRangeCheckDTO(currentTime, ring.getCurrentSegmentTimingInformationCopy(), ring.getCurrentPhaseSegment(), 0, ring));
            lateTimes.put(ring, new SignalRangeCheckDTO(currentTime, ring.getCurrentSegmentTimingInformationCopy(), ring.getCurrentPhaseSegment(), 0, ring));
        }
        //System.out.println(currentTime + " " + futureTime);
        //these mutate earlyTimes and lateTimes in place.
        getSegmentRangeBySingleTimeHelper(earlyTimes, futureTime, (SimConfig.ALLOW_ACTUATION ? true : false));
        getSegmentRangeBySingleTimeHelper(lateTimes, futureTime, false);

        for (int ri : sortedRingIds) {
            RBRing ring = rings.get(ri);
            results.put(ring, ring.getSegmentsByRelativeIndexRangeFromCurrentPhaseSegment(lateTimes.get(ring).index, earlyTimes.get(ring).index));
        }
        return results;
    }

    /**
     * Mutates timeTuples in place to get desired result showing where the
     * earliest or farthest (depending on chooseEarly) segments are in the phase
     * segment sequence. Note, RBRing should protect against rings with 0
     * minimum or maximum time by checking that the minimum time is
     *
     * @param timeTuples
     * @param futureTime
     * @param chooseEarly
     */
    private void getSegmentRangeBySingleTimeHelper(HashMap<RBRing, SignalRangeCheckDTO> timeTuples, double futureTime, boolean chooseEarly) {
        int r = timeTuples.size();
        //max priority queue
        PriorityQueue<SignalRangeCheckComparablePair> waitingOnBarrier = new PriorityQueue<SignalRangeCheckComparablePair>(r, Collections.reverseOrder());
        PriorityQueue<SignalRangeCheckComparablePair> nextBarrierTimes = new PriorityQueue<SignalRangeCheckComparablePair>(r, Collections.reverseOrder());
        Set<RBRing> ringsInNextBarrierTimes = new HashSet<RBRing>();
        HashMap<RBRing, Boolean> done = new HashMap<RBRing, Boolean>();
        int doneCount = 0;

        //System.out.println("Start.++++++++++++++++++++++++++++++++++++++\nChooseEarly: " + chooseEarly +", future time:" + futureTime + ", current time: " + timeTuples.values().iterator().next().time);
        while (doneCount < r) {
            //System.out.println("ITERATION===============================");
            for (SignalRangeCheckDTO srcDTO : timeTuples.values()) {
                if (!done.containsKey(srcDTO.ring)) {
                    done.put(srcDTO.ring, false);
//                    System.out.println("Added key " + srcDTO.ring.getRingId());
//                    System.out.println("Acts for key: " + srcDTO.segment.getTurnDirectionsForPhaseSegment());
//                    System.out.println("Color for key " + srcDTO.segment.getColor().name());
//                    System.out.println("Max: " + srcDTO.segmentInfo.getSimTimeWhenMaxTimeExpires());
//                    System.out.println("Min: " + srcDTO.segmentInfo.getSimTimeWhenMinTimeExpires());
//                    System.out.println("Update: " + srcDTO.segmentInfo.getSimTimeWhenUpdated());
//                    System.out.println("Latest end: " + srcDTO.segmentInfo.getLatestKnownEndTime());
//                    System.out.println("Earliest end: " + srcDTO.segmentInfo.getEarliestKnownEndTime());
//                    System.out.println("Expected end: " + srcDTO.segmentInfo.getSimTimeWhenExpectedEndTimeExpires());
//                    System.out.println("Road: " + srcDTO.segment.getRoad().getName());
                } else if (done.get(srcDTO.ring)) {
                    if (ringsInNextBarrierTimes.contains(srcDTO.ring)) {
                        throw new RuntimeException("This never should have happened. A ring for which the simulation has been completed has been submitted for processing.");
                    }
                    SignalRangeCheckDTO copyOfSrcDTO = new SignalRangeCheckDTO(srcDTO);
                    getTimeOfNextBarrierAndMutateSrcDtoInPlace(copyOfSrcDTO, chooseEarly);
                    nextBarrierTimes.add(new SignalRangeCheckComparablePair((chooseEarly ? copyOfSrcDTO.segmentInfo.getEarliestKnownEndTime() : copyOfSrcDTO.segmentInfo.getLatestKnownEndTime()), srcDTO.ring));
                    ringsInNextBarrierTimes.add(srcDTO.ring);
                    //System.out.println("Finding next barrier for done in block1: " + (chooseEarly ? copyOfSrcDTO.segmentInfo.getEarliestKnownEndTime() : copyOfSrcDTO.segmentInfo.getLatestKnownEndTime()) + " " + srcDTO.ring.getRingId());
                    continue;
                }

                //System.out.println("Segment end time for ring(start) " + srcDTO.ring.getRingId() + ": " + (chooseEarly ? srcDTO.segmentInfo.getEarliestKnownEndTime() : srcDTO.segmentInfo.getLatestKnownEndTime()));
                while ((chooseEarly ? srcDTO.segmentInfo.getEarliestKnownEndTime() : srcDTO.segmentInfo.getLatestKnownEndTime()) < futureTime && !srcDTO.segment.nextPhaseSegment.isBarrier()) {
                    srcDTO.time = (chooseEarly ? srcDTO.segmentInfo.getEarliestKnownEndTime() : srcDTO.segmentInfo.getLatestKnownEndTime());
                    srcDTO.index++;
                    srcDTO.segment = srcDTO.segment.getNextPhaseSegment();
                    RBPhaseSegment seg = srcDTO.segment;
                    //System.out.println("Sim forward for " + srcDTO.ring.getRingId() + " " + srcDTO.time + " " + srcDTO.index);
                    srcDTO.segmentInfo = new HistoricalRBSegmentInformation(srcDTO.ring, seg.getSegmentId(), srcDTO.time, seg.getMinTime(), seg.getMaxTimeAndLock(), (chooseEarly ? seg.getMinTime() : seg.getMaxTimeAndLock()) + srcDTO.time, srcDTO.time, srcDTO.segmentInfo.isEarlyGapoutAllowed(), seg.getColor(), seg.areGapExtensionsAllowed());
                    //System.out.println("Segment end time for ring(step) " + srcDTO.ring.getRingId() + ": " + (chooseEarly ? srcDTO.segmentInfo.getEarliestKnownEndTime() : srcDTO.segmentInfo.getLatestKnownEndTime()));
                }

                if ((chooseEarly ? srcDTO.segmentInfo.getEarliestKnownEndTime() : srcDTO.segmentInfo.getLatestKnownEndTime()) >= futureTime) {
                    done.replace(srcDTO.ring, true);
                    doneCount++;
                    // System.out.println("Ring done: " + srcDTO.ring.getRingId() + " " + srcDTO.index);
                    if (chooseEarly) {
                        srcDTO.index++;
                        //System.out.println("Stepped final index forward for " + srcDTO.ring.getRingId() + " " + srcDTO.index);
                    }

                    SignalRangeCheckDTO copyOfSrcDTO = new SignalRangeCheckDTO(srcDTO);
                    getTimeOfNextBarrierAndMutateSrcDtoInPlace(copyOfSrcDTO, chooseEarly);
                    nextBarrierTimes.add(new SignalRangeCheckComparablePair((chooseEarly ? copyOfSrcDTO.segmentInfo.getEarliestKnownEndTime() : copyOfSrcDTO.segmentInfo.getLatestKnownEndTime()), srcDTO.ring));
                    //System.out.println("Finding next barrier for done in block2: " + (chooseEarly ? copyOfSrcDTO.segmentInfo.getEarliestKnownEndTime() : copyOfSrcDTO.segmentInfo.getLatestKnownEndTime()) + " " + srcDTO.ring.getRingId());
                    ringsInNextBarrierTimes.add(srcDTO.ring);
                } else {
                    waitingOnBarrier.add(new SignalRangeCheckComparablePair((chooseEarly ? srcDTO.segmentInfo.getEarliestKnownEndTime() : srcDTO.segmentInfo.getLatestKnownEndTime()), srcDTO.ring));
                    nextBarrierTimes.add(new SignalRangeCheckComparablePair((chooseEarly ? srcDTO.segmentInfo.getEarliestKnownEndTime() : srcDTO.segmentInfo.getLatestKnownEndTime()), srcDTO.ring));
                    //System.out.println("Ring waiting for barrier " + srcDTO.ring.getRingId() + " " + srcDTO.time + " " + srcDTO.index);
                    ringsInNextBarrierTimes.add(srcDTO.ring);
                }
            }

            if (!waitingOnBarrier.isEmpty()) {
                //double latestTime = waitingOnBarrier.peek().getKey();
                double latestTime = nextBarrierTimes.peek().getKey();

                if (nextBarrierTimes.size() != r) {
                    throw new RuntimeException("Not all times to next barrier were found for all rings.");
                }

                //while (!nextBarrierTimes.isEmpty()) System.out.println(nextBarrierTimes.poll());
                nextBarrierTimes.clear();
                ringsInNextBarrierTimes.clear();
                //handle case where all rings need to advance into a barrier
                if (waitingOnBarrier.size() == r) {
                    while (!waitingOnBarrier.isEmpty()) {
                        SignalRangeCheckComparablePair pair = waitingOnBarrier.poll();
                        SignalRangeCheckDTO srcDTO = timeTuples.get(pair.getValue());
                        srcDTO.time = latestTime;
                        srcDTO.index++;
                        //System.out.println("Step past barrier for " + srcDTO.ring.getRingId() + " " + srcDTO.time + " " + srcDTO.index);
                        srcDTO.segment = srcDTO.segment.getNextPhaseSegment();
                        RBPhaseSegment seg = srcDTO.segment;
                        srcDTO.segmentInfo = new HistoricalRBSegmentInformation(srcDTO.ring, seg.getSegmentId(), srcDTO.time, seg.getMinTime(), seg.getMaxTimeAndLock(), srcDTO.time + (chooseEarly ? seg.getMinTime() : seg.getMaxTimeAndLock()), srcDTO.time, srcDTO.segmentInfo.isEarlyGapoutAllowed(), seg.getColor(), seg.areGapExtensionsAllowed());
                    }
                } else if (waitingOnBarrier.size() > r) {
                    throw new RuntimeException("More rings waiting for barrier than there are rings.");
                } else {
                    //<r which is the case where one ring completed without progressing into the barrier, which means the other rings should have been extended to the same time, too
                    if (latestTime >= futureTime) {
                        while (!waitingOnBarrier.isEmpty()) {
                            SignalRangeCheckComparablePair pair = waitingOnBarrier.poll();
                            SignalRangeCheckDTO srcDTO = timeTuples.get(pair.getValue());
                            //srcDTO.segmentInfo.setSimTimeWhenExpectedEndTimeExpires(???, ??); //didn't bother here, as this information about when the segment ends isn't needed any longer
                            done.replace(srcDTO.ring, true);
                            doneCount++;
                            //System.out.println("Ring done when extending due to barrier: " + srcDTO.ring.getRingId() + " " + srcDTO.index);
                            if (chooseEarly) {
                                srcDTO.index++;
                                //System.out.println("Stepped final index forward when extending due to barrier for " + srcDTO.ring.getRingId() + " " + srcDTO.index);
                            }
                            //we also should be totally done at this point which means that there is no need to do the copying and lookahead for the next barrier at this point
                        }
                    } else {
                        throw new RuntimeException("Segment containing future time found, but the latest time to progress through the barrier isn't past the desired future time. This shouldn't occur.");
                    }
                }
            }
        }
        //System.out.println("Done.------------------------------------");
    }

    private void getTimeOfNextBarrierAndMutateSrcDtoInPlace(SignalRangeCheckDTO copyOfSrcDTO, boolean chooseEarly) {
        while (!copyOfSrcDTO.segment.nextPhaseSegment.isBarrier()) {
            copyOfSrcDTO.time = (chooseEarly ? copyOfSrcDTO.segmentInfo.getEarliestKnownEndTime() : copyOfSrcDTO.segmentInfo.getLatestKnownEndTime());
            copyOfSrcDTO.index = -1;
            copyOfSrcDTO.segment = copyOfSrcDTO.segment.getNextPhaseSegment();
            RBPhaseSegment seg = copyOfSrcDTO.segment;
            copyOfSrcDTO.segmentInfo = new HistoricalRBSegmentInformation(copyOfSrcDTO.ring, seg.getSegmentId(), copyOfSrcDTO.time, seg.getMinTime(), seg.getMaxTimeAndLock(), copyOfSrcDTO.time + (chooseEarly ? seg.getMinTime() : seg.getMaxTimeAndLock()), copyOfSrcDTO.time, copyOfSrcDTO.segmentInfo.isEarlyGapoutAllowed(), seg.getColor(), seg.areGapExtensionsAllowed());
        }
    }

    public double getAvgCycleLength() {
        return movingAverageOfCycleLength;
    }
}
