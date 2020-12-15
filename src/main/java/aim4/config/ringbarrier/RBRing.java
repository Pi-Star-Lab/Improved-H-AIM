package aim4.config.ringbarrier;

import aim4.config.Constants;
import aim4.config.Constants.TurnDirection;
import aim4.config.SimConfig;
import aim4.config.TrafficSignal;
import aim4.im.IntersectionManager;
import aim4.map.Road;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.PriorityQueue;
import java.util.Set;

public class RBRing implements Comparable<RBRing> {

    private static int nextRingId = 0;

    private final LinkedList<RBPhaseSegment> phaseSegments;
    private LinkedList<RBPhaseSegment> representativeSegmentsForPhases;
    private RBPhaseSegment currentSegment;

    private HistoricalRBSegmentInformation currentSegmentTimingInformation;
    private LinkedList<HistoricalRBSegmentInformation> historicalSegmentInformation;
    private final int maxHistoricalLength = 30;

    private boolean comparisonMayBeStale; //flags that the segment end has changed, and therefore entry of this ring in the containing Ring and Barrier's priority queue for updates may be stale.
    private boolean earlyGapoutsPermitted;
    private final int ringId;

    private HashMap<Road, EnumSet<Constants.TurnDirection>> turnDirectionsForRing;
    private int phaseGroupsCount;
    private RBPhaseSegment firstBarrierEndingSegmentInRing;
    private RBPhaseSegment lastBarrierEndingSegmentInRing;

    private LinkedList<Set<Long>> segmentIdsForPhases;

    public RBRing(List<RBPhaseSegment> phaseSegments, boolean shouldEarlyGapout) {
        //Long.MIN_VALUE isn't a valid ID
        currentSegmentTimingInformation = new HistoricalRBSegmentInformation(this, HistoricalRBSegmentInformation.INVALID_ID, 0, 0, 0, 0, -1, false, TrafficSignal.UNKNOWN, false);
        historicalSegmentInformation = new LinkedList<HistoricalRBSegmentInformation>();
        comparisonMayBeStale = false;
        firstBarrierEndingSegmentInRing = null;
        lastBarrierEndingSegmentInRing = null;

        this.earlyGapoutsPermitted = shouldEarlyGapout;
        this.phaseSegments = new LinkedList<RBPhaseSegment>(phaseSegments);

        //these are set on call to bindIMSegments
        currentSegment = null;
        currentSegmentTimingInformation = null;
        //moved to bindIMToSegments
//        if (!this.phaseSegments.isEmpty()) {
//            currentSegment = this.phaseSegments.get(0);
//            //double epoch, double minTime, double maxTime, double expectedEndTime, double gapOut, double simTimeWhenUpdated
//            //end time assumes we're starting at 0
//            currentSegmentTimingInformation = new HistoricalRBSegmentInformation(currentSegment.getSegmentId(), 0, currentSegment.getMinTime(), currentSegment.getMaxTime(), currentSegment.getMinTime(), 0, this.earlyGapoutsPermitted, currentSegment.getColor(), currentSegment.areGapExtensionsAllowed());
//        }

        ringId = nextRingId++;
        if (ringId < 0) {
            throw new RuntimeException("More rings have been created than are representable by the positive numbers in [0, " + Integer.MIN_VALUE + "]. IDs for rings have overflowed.");
        }

        turnDirectionsForRing = new HashMap<Road, EnumSet<Constants.TurnDirection>>();
        representativeSegmentsForPhases = null;
        segmentIdsForPhases = null;

        phaseGroupsCount = 0;
        setUpPhaseInformationFromPhaseSegments(); //set up representativeSegmentsForPhases, final and first ring and barrier, turnDirectionsForRing, phaseGroupsCount, and segmentIdsForPhases
    }

    public void bindIMToSegments(IntersectionManager im) {
        for (RBPhaseSegment seg : phaseSegments) {
            if (!seg.isRedOrYellowSegment() && !seg.setAssociatedIntersectionManager(im)) {
                throw new RuntimeException("Couldn't bind intersection manager to segment: " + seg);
            }
        }

        if (!phaseSegments.isEmpty()) {
            currentSegment = phaseSegments.get(0);
        }
    }

    public void setUpSegmentTimingInformation() {
        //double epoch, double minTime, double maxTime, double expectedEndTime, double gapOut, double simTimeWhenUpdated
        //end time assumes we're starting at 0
        currentSegmentTimingInformation = new HistoricalRBSegmentInformation(this, currentSegment.getSegmentId(), 0, currentSegment.getMinTime(), currentSegment.getMaxTimeAndLock(), currentSegment.getMinTime(), 0, earlyGapoutsPermitted, currentSegment.getColor(), currentSegment.areGapExtensionsAllowed());
        currentSegment.setActiveTimingInfo(currentSegmentTimingInformation, 0);
    }

    //set up representativeSegmentsForPhases, final and first ring and barrier, turnDirectionsForRing, phaseGroupsCount, and segmentIdsForPhases
    private void setUpPhaseInformationFromPhaseSegments() {
        representativeSegmentsForPhases = new LinkedList<RBPhaseSegment>();
        segmentIdsForPhases = new LinkedList<Set<Long>>();
        Set<Long> currentSet = null;
        Set<RBPhaseSegment> leftoversFromCycle = new HashSet<RBPhaseSegment>();
        boolean stopAddingToLeftover = false;

        TrafficSignal currentColor = null;

        double totalMinimumTimeOfSegments = 0;
        boolean shouldCount = true;

        //cursory check that we have a valid green->yellow->red transition, including the end/beginning colors
        if (phaseSegments.isEmpty() || phaseSegments.size() % 3 != 0) {
            throw new RuntimeException("Unable to produce phases in ring, as phase segment does not have the expected number of traffic signals. Please ensure there are more than 0 segments, and that the segments follow a green->tellow->red transition.");
        } else if (phaseSegments.getFirst().getColor() != getNextPhaseSegmentColorForPhaseParserFSM(phaseSegments.getLast().getColor())) {
            throw new RuntimeException("Unable to produce phases in ring, as phase segment has unexpected traffic signal color. The first and last signal colors do not align to the expected green->tellow->red transition.");
        }

        RBPhaseSegment segmentRepresentingPhase = null;
        //todo, some of this logic can be squashed together maybe?
        //get the representative segments, and verify the green->yellow->red transition always happens as expected. The single wrap around case is handled in the checks above.
        for (RBPhaseSegment phaseSegment : phaseSegments) {
            //Block 1: handle sorting out the number of phase groupings and the turn directions for the ring
            totalMinimumTimeOfSegments += phaseSegment.getMinTimeAssumingNoSegmentToHoldOnTo();
            if (turnDirectionsForRing.keySet().contains(phaseSegment.getRoad())) {
                turnDirectionsForRing.get(phaseSegment.getRoad()).addAll(phaseSegment.getTurnDirectionsForPhaseSegment());
            } else {
                turnDirectionsForRing.put(phaseSegment.getRoad(), EnumSet.noneOf(Constants.TurnDirection.class));
            }

            if (phaseSegment.isBarrier()) {
                shouldCount = true;
            } else if (!phaseSegment.isBarrier() && shouldCount) {
                shouldCount = false;
                ++phaseGroupsCount;
            }

            //Block 2: sort out the segments which begin each group of green->yellow->red steppings as well as figure out where the first and last barriers end
            if (currentColor == null) { //first phase checked
                currentColor = phaseSegment.getColor();
                if (currentColor == TrafficSignal.GREEN) {
                    representativeSegmentsForPhases.add(phaseSegment);
                    phaseSegment.setSegmentRepresentingPhase(phaseSegment);
                    segmentRepresentingPhase = phaseSegment;
                    stopAddingToLeftover = true;
                    currentSet = new HashSet<Long>();
                    currentSet.add(phaseSegment.getSegmentId());
                } else {
                    leftoversFromCycle.add(phaseSegment);
                    firstBarrierEndingSegmentInRing = (phaseSegment.isBarrier() && phaseSegment.getColor() == TrafficSignal.RED && firstBarrierEndingSegmentInRing == null ? phaseSegment : firstBarrierEndingSegmentInRing);
                    lastBarrierEndingSegmentInRing = (phaseSegment.isBarrier() && phaseSegment.getColor() == TrafficSignal.RED ? phaseSegment : lastBarrierEndingSegmentInRing);
                }
            } else if (phaseSegment.getColor() == getNextPhaseSegmentColorForPhaseParserFSM(currentColor)) {
                currentColor = phaseSegment.getColor();
                if (currentColor == TrafficSignal.GREEN) {
                    representativeSegmentsForPhases.add(phaseSegment);
                    phaseSegment.setSegmentRepresentingPhase(phaseSegment);
                    segmentRepresentingPhase = phaseSegment;
                    stopAddingToLeftover = true;
                    if (currentSet != null) {
                        segmentIdsForPhases.add(Collections.unmodifiableSet(currentSet));
                    }
                    currentSet = new HashSet<Long>();
                    currentSet.add(phaseSegment.getSegmentId());
                } else if (!stopAddingToLeftover) {
                    leftoversFromCycle.add(phaseSegment);
                    firstBarrierEndingSegmentInRing = (phaseSegment.isBarrier() && phaseSegment.getColor() == TrafficSignal.RED && firstBarrierEndingSegmentInRing == null ? phaseSegment : firstBarrierEndingSegmentInRing);
                    lastBarrierEndingSegmentInRing = (phaseSegment.isBarrier() && phaseSegment.getColor() == TrafficSignal.RED ? phaseSegment : lastBarrierEndingSegmentInRing);
                } else { //stopAddingToLeftover
                    if (currentSet == null) {
                        throw new RuntimeException("setUpPhaseInformationFromPhaseSegments was modified such that an element was about to be added to a null set. Please review the function's logic.");
                    }
                    phaseSegment.setSegmentRepresentingPhase(segmentRepresentingPhase);
                    currentSet.add(phaseSegment.getSegmentId());
                    firstBarrierEndingSegmentInRing = (phaseSegment.isBarrier() && phaseSegment.getColor() == TrafficSignal.RED && firstBarrierEndingSegmentInRing == null ? phaseSegment : firstBarrierEndingSegmentInRing);
                    lastBarrierEndingSegmentInRing = (phaseSegment.isBarrier() && phaseSegment.getColor() == TrafficSignal.RED ? phaseSegment : lastBarrierEndingSegmentInRing);
                }
            } else {
                throw new RuntimeException("Unable to produce phases in ring, as phase segment has unexpected traffic signal. Current color: " + currentColor.name() + ". Expected next color: " + getNextPhaseSegmentColorForPhaseParserFSM(currentColor).name() + ". Next color: " + phaseSegment.getColor().name());
            }
        }

        if (totalMinimumTimeOfSegments <= 0) {
            //this could be fixed by regearing the lookahead to terminate early if it realizes that ALL segments will be included, but with the current early then late setup, this isn't possible to handle.
            throw new IllegalArgumentException("A ring may not have a total minimum round time of 0. This makes any timebased lookahead (such as an IM does when evaluating reservation requests in the future) infeasible with current implementation.");
        }

        //if there's anything in the leftovers from cycling around the end of the list, add them to the last set and add the set
        if (currentSet != null) {
            for (RBPhaseSegment phaseSegment : leftoversFromCycle) {
                phaseSegment.setSegmentRepresentingPhase(segmentRepresentingPhase);
                currentSet.add(phaseSegment.getSegmentId());
            }
            segmentIdsForPhases.add(Collections.unmodifiableSet(currentSet));
        }
    }

    //returns the next traffic signal color we expect to see given a current signal in order to parse phases from the phase segments
    private TrafficSignal getNextPhaseSegmentColorForPhaseParserFSM(TrafficSignal currentColor) {
        switch (currentColor) {
            case GREEN:
                return TrafficSignal.YELLOW;
            case YELLOW:
                return TrafficSignal.RED;
            case RED:
                return TrafficSignal.GREEN;
            default:
                return TrafficSignal.UNKNOWN;
        }
    }

    //todo memoize this if it ever gets used more than once
    public LinkedList<Set<Constants.TurnDirection>> getTurnDirectionsForPhases() {
        LinkedList<Set<Constants.TurnDirection>> retList = new LinkedList<Set<Constants.TurnDirection>>();
        for (RBPhaseSegment phaseSegment : representativeSegmentsForPhases) {
            retList.add(phaseSegment.getTurnDirectionsForPhaseSegment());
        }
        return retList;
    }

    //todo memoize this if it ever gets used more than once
    public LinkedList<Double> getStreetDirectionsForPhases(IntersectionManager im) {
        if (im == null) {
            throw new IllegalArgumentException("The intersection manager may not be null when calling getStreetDirectionsForPhases.");
        }

        LinkedList<Double> retList = new LinkedList<Double>();
        for (RBPhaseSegment phaseSegment : representativeSegmentsForPhases) {
            Road roadForSegment = phaseSegment.getRoad();
            retList.add(im.getIntersection().getEntryHeading(roadForSegment.getIndexLane()));
        }
        return retList;
    }

    //todo memoize this if it ever gets used more than once
    public Road getFirstRoadInRing() {
        for (RBPhaseSegment phase : phaseSegments) {
            Road rd = phase.getRoad();
            if (rd != null) {
                return rd;
            }
        }
        return null;
    }

    public List<RBPhaseSegment> getPhaseSegments() {
        return Collections.unmodifiableList(phaseSegments);
    }

    public int getRingId() {
        return ringId;
    }

    public static int getNumberOfTotalRingsCreated() {
        return (nextRingId < 0 ? Integer.MAX_VALUE : nextRingId);
    }

    /**
     * Gets the current phase segment based on the latest update that was
     * performed.
     *
     * @return The segment that was set as current by the latest update round.
     */
    public RBPhaseSegment getCurrentPhaseSegment() {
        return currentSegment;
    }

    public boolean advanceSegmentIfNotCrossingIntoBarrier(double currentTime, boolean shouldMarkStale, ActuationTracker at) {
        //check if the next phase segment is a barrier while the current one isn't, this includes a check incase we're at the end
        if (currentSegment.getNextPhaseSegment().isBarrier() && !currentSegment.isBarrier()) {
            return false;
        } else {
            forceAdvanceSegment(currentTime, shouldMarkStale, at, currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires());
            return true;
        }
    }

    //todo, this is likely very ineffecient
    /**
     * Forces the advancement of a segment using the passed epoch time as the
     * next segment's epoch time
     *
     * @param currentTime
     * @param shouldMarkStale
     * @param at
     * @param epochTime
     */
    protected void forceAdvanceSegment(double currentTime, boolean shouldMarkStale, ActuationTracker at, double epochTime) {
        /*if (!currentSegment.isRedOrYellowSegment()) {
            if (currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires() >= currentSegmentTimingInformation.getSimTimeWhenMaxTimeExpires()) {
                System.out.println("Maxout");
            }
        }*/

        comparisonMayBeStale = shouldMarkStale;
        currentSegment.unlockForSegmentAdjustmentsAndClearActiveTimingCache();
        currentSegment = currentSegment.getNextPhaseSegment();
        if (!currentSegment.isLocked()) {
            currentSegment.lockForSegmentAdjustments();
        }

        while (historicalSegmentInformation.size() >= maxHistoricalLength) {
            historicalSegmentInformation.remove(); //remove oldest element
        }
        HistoricalRBSegmentInformation tempSegmentInfo = currentSegmentTimingInformation;

        //double epoch, double minTime, double maxTime, double expectedEndTime, double gapOut, double simTimeWhenUpdated
        currentSegmentTimingInformation = new HistoricalRBSegmentInformation(this, currentSegment.getSegmentId(), epochTime, currentSegment.getMinTime(), currentSegment.getMaxTimeAndLock(), epochTime + currentSegment.getMinTime(), currentTime, earlyGapoutsPermitted, currentSegment.getColor(), currentSegment.areGapExtensionsAllowed());
        currentSegment.setActiveTimingInfo(currentSegmentTimingInformation, currentTime);
        updateValuesBasedOnActuations(currentSegmentTimingInformation.getEpoch(), at);
        //System.out.print("******" + currentSegment.getRoad().getName() + ", " + currentSegment.getTurnDirectionsForPhaseSegment() + ", " + currentSegment.getColor() + " " + currentSegmentTimingInformation.getSimTimeWhenMinTimeExpires());
        tempSegmentInfo.setSimTimeWhenUpdated(currentTime);
        historicalSegmentInformation.add(tempSegmentInfo);
    }

    //todo, again, likely very ineffecient
    public void logActuation(double currentTime, ActuationTracker at) {
        currentSegmentTimingInformation.setSimTimeWhenUpdated(currentTime);
        //todo, this work actuation checking work was redone under the assumption that a future actuation could fill a gap in previous actuations to cause later actuations to be come linked in the chain of actuations/extensions...with an early gapout this isn't neccessarily possible. Recheck this logic
        updateValuesBasedOnActuations(currentTime, at);
    }

    private void updateValuesBasedOnActuations(double beginTime, ActuationTracker at) {
        double minEndTime = currentSegmentTimingInformation.getSimTimeWhenMinTimeExpires();
        double maxEndTime = currentSegmentTimingInformation.getSimTimeWhenMaxTimeExpires();

        if (currentSegmentTimingInformation.areGapExtensionsAllowed()) {
            PriorityQueue<ActuationExitPair> actuationsAndExits = at.getActuations(beginTime, maxEndTime, currentSegment.getRoad(), currentSegment.getTurnDirectionsForPhaseSegment());

            Double time = null;
            if (!actuationsAndExits.isEmpty()) {
                time = currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires();
            }

            while (!actuationsAndExits.isEmpty() && currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires() != maxEndTime) { //check to see if enough actuations have been logged to guarantee a transition to the next phase
                ActuationExitPair aePair = actuationsAndExits.poll();
                double actuation = aePair.getActuationTime();

                if (actuation < currentSegmentTimingInformation.getEpoch()) {
                    continue;
                }

                if (actuation < minEndTime) { //check for how long to extend the time if the actuation occurs before the minimum time has passed
                    comparisonMayBeStale = true;
                    currentSegmentTimingInformation.addActuationIfTimeAdvanced(actuation);
                    //the math.min ensures that a gap extension that is longer than the difference between the max and min time doesn't push the segment past its max time.
                    currentSegmentTimingInformation.setSimTimeWhenExpectedEndTimeExpires(Math.min(currentSegmentTimingInformation.getSimTimeWhenMaxTimeExpires(), minEndTime + Math.max(0, currentSegment.getGapTime() - (minEndTime - actuation))));
                } else if (actuation > currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires()) { //actuation is past when the next gapout would occur, so all actuations including and after this one don't affect the timer unless we early gapout
                    break;
                    /*if (!earlyGapoutsPermitted) {
                        break;
                    } else {
                        double previousEndTime = currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires();
                        //we're moving the end time backwards because we got new information about when another vehicle will arrive, but we can't move later than the current end or earlier than the min time (which is covered by the check in the first if).
                        currentSegmentTimingInformation.earlyGapoutIfPossible();
                        if (previousEndTime != currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires()) {
                            comparisonMayBeStale = true;
                            System.out.println("EARLY GAPOUT: " + currentSegment.getRoad().getName() + " " + currentSegment.getTurnDirectionsForPhaseSegment() + ". Old end time was: " + previousEndTime + " New endtime is: " + currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires());
                        }
                        break; //there's no more point in looking at actuations that are further in the future, so just break
                    }*/
                } else if (actuation < currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires() && actuation < maxEndTime) { //check how long to extend the time if the gap timer should be reset due to an actuation
                    comparisonMayBeStale = true;
                    currentSegmentTimingInformation.addActuationIfTimeAdvanced(actuation);
                    currentSegmentTimingInformation.setSimTimeWhenExpectedEndTimeExpires(Math.max(currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires(), Math.min(actuation + currentSegment.getGapTime(), currentSegmentTimingInformation.getSimTimeWhenMaxTimeExpires())));
                }

                if (currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires() >= maxEndTime) { //break early if we hit the end of the phase segment's max time
                    break;
                }
            }

//            if (time != null && currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires() - time > 0) {
//                System.out.println("Actuation:  ID: " + currentSegmentTimingInformation.getId() + ". Old end time was: " + time + " New endtime is: " + currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires());
//            }
        }
        if (currentSegment.getOtherSegmentsMightHold()) {
            double timeToTarget = currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires();
            //tell all the waiting segments to wait until at least when this segment will end to end themselves
            for (RBPhaseSegment seg : currentSegment.getSetOfSegmentsHolding()) {
                //all waiting segments should have max times >= this segment's.
                seg.requestThatTimingInfoUpdateToTime(timeToTarget, beginTime);
            }
        }
    }

    public void resetCurrentEarlyGapoutTimerForPhaseHistorical(double arrivalTime) {
        currentSegmentTimingInformation.resetEarlyGapTimeToExpectedEndTimeIfWaitingOnVehicle(arrivalTime);
    }

    public boolean flagEarlyGapout(double currentTime) {
        double maxEndTime = currentSegmentTimingInformation.getSimTimeWhenMaxTimeExpires();
        if (currentTime >= currentSegmentTimingInformation.getSimTimeWhenMinTimeExpires() && currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires() < maxEndTime) {
            double previousEndTime = currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires();
            currentSegmentTimingInformation.setSimTimeWhenUpdated(currentTime);
            currentSegmentTimingInformation.earlyGapoutIfPossible();
            if (previousEndTime != currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires()) {
                comparisonMayBeStale = true;
                //System.out.println("NO MORE VEHICLES EARLY GAPOUT: " + currentSegment.getRoad().getName() + " " + currentSegment.getTurnDirectionsForPhaseSegment() + ". Old end time was: " + previousEndTime + " New endtime is: " + currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires());
                return true;
            }
        }
        return false;
    }

    public int getPhaseGroupsCount() {
        return phaseGroupsCount;
    }

    public Double getCurrentSegmentExpectedEnd() {
        return currentSegmentTimingInformation.getSimTimeWhenExpectedEndTimeExpires();
    }

    public HistoricalRBSegmentInformation getCurrentSegmentTimingInformationCopy() {
        return new HistoricalRBSegmentInformation(currentSegmentTimingInformation);
    }

    public boolean comparisonMayBeStale() {
        return comparisonMayBeStale;
    }

    /**
     * This is only designed to reset the comparisonMayBeStale flag when the
     * ring object is contained within one priority queue for updates within a
     * single ring and barrier.
     */
    protected void resetComparisonMayBeStale() {
        comparisonMayBeStale = false;
    }

    /**
     * Used to help with ordering in priority queues that are time based, such
     * that the rings whose phases end first get addressed first. Used in
     * combination with resetComparisonMayBeStale() to make sure extensions are
     * handled properly.
     *
     * @param t
     * @return
     */
    @Override
    public int compareTo(RBRing t) {
        return getCurrentSegmentExpectedEnd().compareTo(t.getCurrentSegmentExpectedEnd());
    }

    public boolean ringHandlesDesiredActions(Road rd, Set<Constants.TurnDirection> tds) {
        if (turnDirectionsForRing.keySet().contains(rd)) {
            return !Collections.disjoint(tds, turnDirectionsForRing.get(rd));
        } else {
            return false;
        }
    }

    public LinkedList<HistoricalRBSegmentInformation> getAndClearHistoricalTimingInformation() {
        LinkedList<HistoricalRBSegmentInformation> infoToReturn = historicalSegmentInformation;
        historicalSegmentInformation = new LinkedList<HistoricalRBSegmentInformation>();
        return infoToReturn;
    }

    //todo, may need to scale back the number of times this gets called...need to check.
    public void updateUpdateTimeOfSegmentTimingInformation(double currentTime) {
        currentSegmentTimingInformation.setSimTimeWhenUpdated(currentTime);
    }

    public List<Set<Long>> getSegmentIdsForPhases() {
        return Collections.unmodifiableList(segmentIdsForPhases);
    }

    /**
     *
     * @param startInclusive relative index to start with. 0 means the current
     * phase segment. Will be processed as (given value)%(number of phase
     * segments).
     * @param endExclusive relative index to end with, exclusive. Will be
     * processed as (given value)%(number of phase segments).
     * @return The set of RBSegmentReadOnlyNoLockingView within the range
     * provided
     */
    public LinkedHashSet<RBSegmentReadOnlyNoLockingView> getSegmentsByRelativeIndexRangeFromCurrentPhaseSegment(int startInclusive, int endExclusive) {
        if (startInclusive < 0 || endExclusive < 0) {
            throw new IllegalArgumentException("Both range values must be positive. Negative indices are not supported at this time.");
        } else if (SimConfig.ALLOW_ACTUATION && startInclusive >= endExclusive) {
            throw new IllegalArgumentException("The start range value must be strictly less than the end range value, start: " + startInclusive + ", end: " + endExclusive);
        }
        LinkedHashSet<RBSegmentReadOnlyNoLockingView> results = new LinkedHashSet<RBSegmentReadOnlyNoLockingView>();
        int realStart = startInclusive % phaseSegments.size();
        int realEnd = endExclusive % phaseSegments.size();
        RBSegmentReadOnlyNoLockingView segment = new RBSegmentReadOnlyNoLockingView(currentSegment);
        for (int i = 0; (SimConfig.ALLOW_ACTUATION ? i < realEnd : i <= realEnd); ++i) {
            if (i >= realStart) {
                if (results.contains(segment)) {
                    throw new RuntimeException("A segment would have been added twice based on the range values given. This means a vehicle is requesting a reservation at least a full cycle in advance, which is not allowed. start: " + startInclusive + ", end: " + endExclusive);
                }
                results.add(segment);
            }
            segment = segment.getRBSegmentReadOnlyNoLockingViewForNextPhaseSegment();
        }
        return results;
    }

    /**
     * @param id
     * @return true if the phase segment id is the ending segment id of a
     * barrier which is the last barrier in the linked list representing the
     * ring
     */
    public boolean phaseIsLastBarrierPhaseInCycle(long id) {
        return id == lastBarrierEndingSegmentInRing.getSegmentId();
    }

    /**
     * @param id
     * @return true if the phase segment id is the ending segment id of a
     * barrier which is the first barrier in the linked list representing the
     * ring
     */
    public boolean phaseIsFirstBarrierPhaseInCycle(long id) {
        return id == firstBarrierEndingSegmentInRing.getSegmentId();
    }

    double getCurrentSegmentEpoch() {
        return currentSegmentTimingInformation.getEpoch();
    }

    public boolean areEarlyGapoutsPermitted() {
        return earlyGapoutsPermitted;
    }

    /**
     * TODO This function is a bit of a sin as it kind of exposes the the
     * comparisonIsStale flag to being set externally, which isn't ideal. This
     * really should just be routed through the phase segment itself and
     * controlled by views.
     */
    public void setComparisonAsStale() {
        comparisonMayBeStale = true;
    }
}
