package aim4.config.ringbarrier;

import aim4.config.Resources;
import aim4.config.SimConfig;
import aim4.config.TrafficSignal;

/**
 * Object to help with managing the historical tracking of signal behavior. Used
 * by Ring objects and then thrown away (actually stored temporarily, but never
 * used again) when a phase transition happens, retrieved by the
 * ActuatedSignalVisualizationRepresentation for use in drawing. This is
 * effectively another type of view for phase segments.
 *
 */
public class HistoricalRBSegmentInformation {

    public static final long INVALID_ID = Long.MIN_VALUE;

    private final RBRing owningRing;
    private final long id;
    private final double epoch;
    private final double minTime;
    private final double maxTime;
    private Double simTimeIfEarlyGapout;
    private double expectedEndTime;
    private double simTimeWhenUpdated;
    private boolean gapoutExtensionAllowed;
    private boolean earlyGapoutAllowed, gappedOutEarly;
    private TrafficSignal signalColor;
    private int numberOfActuations;
    private double lastActuationTime;

    //Long.MIN_VALUE isn't a valid ID
    public HistoricalRBSegmentInformation(RBRing owningRing, long id, double epoch, double minTime, double maxTime, double expectedEndTime, double simTimeWhenUpdated, boolean earlyGapoutAllowed, TrafficSignal signalColor, boolean gapoutAllowed) {
        this.owningRing = owningRing;
        this.id = id;
        this.epoch = epoch;
        this.minTime = minTime;
        this.maxTime = maxTime;
        this.simTimeIfEarlyGapout = this.epoch + this.minTime;
        this.expectedEndTime = (SimConfig.ALLOW_ACTUATION ? expectedEndTime : this.epoch + this.maxTime);
        this.simTimeWhenUpdated = simTimeWhenUpdated;
        this.earlyGapoutAllowed = earlyGapoutAllowed;
        this.gappedOutEarly = false;
        this.signalColor = signalColor;
        numberOfActuations = 0;
        lastActuationTime = epoch;
        gapoutExtensionAllowed = gapoutAllowed;
    }

    public HistoricalRBSegmentInformation(HistoricalRBSegmentInformation si) {
        this.owningRing = si.owningRing;
        this.id = si.id;
        this.epoch = si.epoch;
        this.minTime = si.minTime;
        this.maxTime = si.maxTime;
        this.simTimeIfEarlyGapout = si.simTimeIfEarlyGapout;
        this.expectedEndTime = si.expectedEndTime;
        this.simTimeWhenUpdated = si.simTimeWhenUpdated;
        this.earlyGapoutAllowed = si.earlyGapoutAllowed;
        this.gappedOutEarly = si.gappedOutEarly;
        this.signalColor = si.signalColor;
        this.numberOfActuations = si.numberOfActuations;
        this.lastActuationTime = si.lastActuationTime;
        this.gapoutExtensionAllowed = si.gapoutExtensionAllowed;
    }

    public long getId() {
        return id;
    }

    public double getEpoch() {
        return epoch;
    }

    public double getMinTime() {
        return minTime;
    }

    public double getMaxTime() {
        return maxTime;
    }

    public double getEarliestKnownEndTime() {
        Double gapoutStandIn = (earlyGapoutAllowed ? simTimeIfEarlyGapout : null);
        if (gapoutStandIn != null && gapoutStandIn < simTimeWhenUpdated) {
            //early gapout time became stale, reset it.
            simTimeIfEarlyGapout = expectedEndTime;
        }

        if (simTimeWhenUpdated <= getSimTimeWhenMinTimeExpires()) {
            return getSimTimeWhenMinTimeExpires();
        } else if (gapoutStandIn != null && simTimeWhenUpdated <= gapoutStandIn) { //only consider an early gapout if the time it should have occured hasn't yet passed.
            //this min operation prevents the minimum time from being longer in the case that the earliest allowed gapout is beyond the current expected end time
            //the current implementation for finding the early gapout time is to look at the exit time for the last actuation. This means that the exit time could conceivably be beyond the current expected end. The gapout extension may be less than the time it will take for a vehicle to exit, for instance.
            return Math.min(gapoutStandIn, getLatestOfExpectedEndTimeAndUpdateTime());
        } else if (simTimeWhenUpdated <= expectedEndTime) {
            return expectedEndTime;
        } else if (simTimeWhenUpdated > expectedEndTime) {
            return simTimeWhenUpdated;
        } else {
            throw new RuntimeException("Couldn't get earliest known end time. This error shouldn't have been possible, so here's the state the caused it:\n"
                    + "Sim time when updated: " + simTimeWhenUpdated + "\n"
                    + "Sim time when min expires: " + getSimTimeWhenMinTimeExpires() + "\n"
                    + "Early gapout sim time: " + simTimeIfEarlyGapout + "\n"
                    + "Expected end time: " + expectedEndTime + "\n"
                    + "Expected end time: " + expectedEndTime + "\n"
                    + "Max end time: " + getSimTimeWhenMaxTimeExpires());
        }
    }

    public double getLatestOfExpectedEndTimeAndUpdateTime() {
        return Math.max(expectedEndTime, simTimeWhenUpdated);
    }
    
    public void resetEarlyGapTimeToExpectedEndTimeIfWaitingOnVehicle(double arrivalTime) {
        if (simTimeIfEarlyGapout == arrivalTime) {
            simTimeIfEarlyGapout = expectedEndTime;
        }
    }

    /**
     *
     * @return the max end time or the most recent update time to this object if
     * the update time is past the expected end time
     */
    public double getLatestKnownEndTime() {
        return Math.max(getSimTimeWhenMaxTimeExpires(), simTimeWhenUpdated);
    }

    public double getSimTimeWhenExpectedEndTimeExpires() {
        return (SimConfig.ALLOW_ACTUATION ? expectedEndTime : getSimTimeWhenMaxTimeExpires());
    }

    public double getSimTimeWhenUpdated() {
        return simTimeWhenUpdated;
    }

    public double getSimTimeWhenMinTimeExpires() {
        return epoch + minTime;
    }

    public double getSimTimeWhenMaxTimeExpires() {
        return epoch + maxTime;
    }

    public double getTimeElapsedSinceEpoch() {
        return simTimeWhenUpdated - epoch;
    }

    public boolean isGappedOutEarly() {
        return gappedOutEarly;
    }

    /**
     * Updates the time stamp at which the segment is expected to end. Also
     * updates the early gapout rewind time. Must be >= epoch +minTime. Not hard
     * capped to epoch+maxTime as a signal could be extended beyond its max time
     * by another ring being extended just before a barrier.
     *
     * @param expectedEndTime
     */
    public void setSimTimeWhenExpectedEndTimeExpires(double expectedEndTime) {
        if (!gappedOutEarly) {
            this.expectedEndTime = Math.max(expectedEndTime, getSimTimeWhenMinTimeExpires());
        }

    }

    /**
     * Rewinds the expected end time to the rewind time for an early gapout, but
     * only if the expected end time has not already met or surpassed the
     * maximum time of the segment and the last time updated has not passed the
     * gapout time.
     */
    public void earlyGapoutIfPossible() {
        /* System.out.println("Rewind=====================");
        System.out.println(earlyGapoutAllowed);
        System.out.println(expectedEndTime);
        System.out.println(epoch + maxTime);
        System.out.println(getLatestKnownEndTime());
        System.out.println(simTimeIfEarlyGapout);
        System.out.println(simTimeWhenUpdated);*/
        if (!gappedOutEarly && earlyGapoutAllowed && expectedEndTime <= epoch + maxTime && getLatestOfExpectedEndTimeAndUpdateTime() > simTimeIfEarlyGapout && simTimeWhenUpdated <= simTimeIfEarlyGapout) {
            gappedOutEarly = true;
            expectedEndTime = simTimeIfEarlyGapout;
            //expectedEndTime = Math.min(Math.max(getSimTimeWhenMinTimeExpires(), simTimeWhenUpdated), getSimTimeWhenMaxTimeExpires());
            System.out.println("+++Did early gapout");
        }
        //System.out.println("Rewind Done================");
    }

    /**
     * Updates the time stamp at which the segment was last updated. May never
     * be reduced.
     *
     * @param simTimeWhenUpdated
     */
    public void setSimTimeWhenUpdated(double simTimeWhenUpdated) {
        this.simTimeWhenUpdated = Math.max(simTimeWhenUpdated, this.simTimeWhenUpdated);
    }

    public boolean isEarlyGapoutAllowed() {
        return earlyGapoutAllowed;
    }

    public TrafficSignal getColor() {
        return signalColor;
    }

    public boolean addActuationIfTimeAdvanced(double actuationTime) {
        if (actuationTime > lastActuationTime && actuationTime <= getSimTimeWhenMaxTimeExpires()) {
            simTimeIfEarlyGapout = actuationTime;
            ++this.numberOfActuations;
            return true;
        }
        return false;
    }

    /**
     * Will report the number of unique times at which actuations were
     * registered. This means that if two actuations occur at the same time,
     * only one will be counted.
     *
     * @return
     */
    public int getNumberOfActuationsWithUniqueTimes() {
        return numberOfActuations;
    }

    /**
     * Will report the last unique time at which an actuation was registered.
     * This means that if two actuations occur at t
     *
     * @return
     */
    public double getLastActuationTimeThatIsUnique() {
        return lastActuationTime;
    }

    boolean areGapExtensionsAllowed() {
        return gapoutExtensionAllowed;
    }
    
    /**
     * TODO
     * This function is a bit of a sin as it kind of exposes the ring, which isn't ideal. This really should just be routed through the phase segment itself and controlled by views.
     */
    public void setComparisonAsStaleForOwningRing() {
        owningRing.setComparisonAsStale();
    }
}
