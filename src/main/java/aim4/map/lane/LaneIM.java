/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package aim4.map.lane;

import aim4.config.Constants;
import aim4.config.SimConfig;
import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

import aim4.im.IntersectionManager;
import aim4.map.Road;
import aim4.util.LimitedPairImplementation;
import aim4.util.Util;
import aim4.vehicle.AutoVehicleDriverView;
import aim4.vehicle.VehicleSimView;
import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;

/**
 * The lane and intersection manager relationship.
 */
public class LaneIM {

    /////////////////////////////////
    // PRIVATE FIELDS
    /////////////////////////////////
    /**
     * The lane
     */
    private final Lane lane;

    /**
     * A map from normalized distances of exit points to intersection managers.
     */
    private final SortedMap<Double, IntersectionManager> intersectionManagers
            = new TreeMap<Double, IntersectionManager>();

    /**
     * Map of distance in lane to vehicles in the lane.
     */
    private SortedMap<Double, VehicleSimView> vehiclesInLane = new TreeMap<Double, VehicleSimView>();
    /**
     * Map of distance along lane to count of vehicles before the next
     * intersection
     */
    private TreeMap<Double, Integer> vehiclesCountCache = new TreeMap<Double, Integer>();

    /**
     * Memoization cache for {@link
     * #nextIntersectionManager(IntersectionManager im)}.
     */
    private Map<IntersectionManager, IntersectionManager> memoGetSubsequentIntersectionManager = null;
    /**
     * Map of vehicle type to...map of lane to...Set of TurnDirections of
     * allowed actions by intersection.
     */
    private final Map<SimConfig.VEHICLE_TYPE, HashMap<IntersectionManager, Set<Constants.TurnDirection>>> laneActionsByVehicleType = new EnumMap<SimConfig.VEHICLE_TYPE, HashMap<IntersectionManager, Set<Constants.TurnDirection>>>(SimConfig.VEHICLE_TYPE.class);
    /**
     * Map of lane to map of TurnDirections to exiting lane.
     */
    private final HashMap<IntersectionManager, HashMap<Constants.TurnDirection, Lane>> laneToLaneByAction = new HashMap<IntersectionManager, HashMap<Constants.TurnDirection, Lane>>();

    /**
     * Whether with traffic turns on red should be allowed
     */
    private boolean allowWithTrafficTurnsOnRed;

    /**
     * Used to keep track of vehicles arriving
     */
    private HashMap<IntersectionManager, HashMap<AutoVehicleDriverView, Double>> imToArrivalsMap;

    /**
     * Used to keep track of vehicles arriving
     */
    private HashMap<IntersectionManager, Queue<LimitedPairImplementation<Double, Constants.TurnDirection>>> imToTDArrivalsMap;

    /**
     * Keeps track of the number of vehicles that have taken each action from
     * this lane
     */
    private HashMap<IntersectionManager, HashMap<Constants.TurnDirection, Integer>> countForTDsInLane;

    /**
     * Used to check if a vehicle seems to have logged as arriving and slipped
     * away without being logged as exiting for turn movement tracking purposes
     */
    private Queue<Double> timesLeftForWhichToLogTurnDirections;

    /**
     * Time in seconds to keep a TD count
     */
    private double timeToKeepTDEntry;

    /////////////////////////////////
    // CONSTRUCTORS
    /////////////////////////////////
    /**
     * Create a lane and intersection manager relationship object.
     *
     * @param lane the lane.
     */
    public LaneIM(Lane lane) {
        allowWithTrafficTurnsOnRed = false;
        imToArrivalsMap = new HashMap<IntersectionManager, HashMap<AutoVehicleDriverView, Double>>();
        imToTDArrivalsMap = new HashMap< IntersectionManager, Queue<LimitedPairImplementation<Double, Constants.TurnDirection>>>();
        countForTDsInLane = new HashMap<IntersectionManager, HashMap<Constants.TurnDirection, Integer>>();
        timesLeftForWhichToLogTurnDirections = new LinkedList<Double>();
        timeToKeepTDEntry = 3600;
        this.lane = lane;
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    // intersection manager
    /**
     * Register an {@link IntersectionManager} with this Lane. If the Lane does
     * not intersect the area controlled by the IntersectionManager, it has no
     * effect. Otherwise, the IntersectionManager is stored and returned under
     * certain circumstances by the <code>nextIntersectionManager</code> method.
     *
     * @param im the IntersectionManager to register
     */
    public void registerIntersectionManager(IntersectionManager im) {
        // Only do this if this lane is managed by this intersection
        if (im.manages(lane)) {
            // Reset this cache
            memoGetSubsequentIntersectionManager = null;
            // Find out where this lane exits the intersection
            Point2D exitPoint = im.getIntersection().getExitPoint(lane);
            // If it's null, that means it doesn't exit.
            if (exitPoint == null) {
                exitPoint = lane.getEndPoint();
            }
            double normalizedDistanceToExit
                    = lane.normalizedDistanceAlongLane(exitPoint);
            // Add the normalized distance to the exit point to the map
            // that gives us the "next intersection" for any point in the lane.
            intersectionManagers.put(normalizedDistanceToExit, im);
            for (SimConfig.VEHICLE_TYPE vType : SimConfig.VEHICLE_TYPE.values()) {
                HashMap<IntersectionManager, Set<Constants.TurnDirection>> map = new HashMap<IntersectionManager, Set<Constants.TurnDirection>>();
                map.put(im, Collections.EMPTY_SET);
                laneActionsByVehicleType.put(vType, map);
            }
            laneToLaneByAction.put(im, new HashMap<Constants.TurnDirection, Lane>(Constants.TurnDirection.values().length));
        }
    }

    /**
     * Get the first IntersectionManager that this Lane, or any Lane it leads
     * into enters. Recursively searches through all subsequent Lanes.
     *
     * @return the first IntersectionManager this Lane, or any Lane it leads
     * into enters
     */
    public IntersectionManager firstIntersectionManager() {
        if (intersectionManagers.isEmpty()) {
            if (lane.hasNextLane()) {
                return lane.getNextLane().getLaneIM().firstIntersectionManager();
            }
            return null;
        }
        return intersectionManagers.get(intersectionManagers.firstKey());
    }

    /**
     * Get the distance from the start of this Lane to the first
     * IntersectionManager that this Lane, or any Lane it leads into intersects.
     * Recursively searches through all subsequent Lanes. Returns
     * <code>Double.MAX_VALUE</code> if no such IntersectionManager exists.
     *
     * @return the distance from the start of this Lane to the first
     * IntersectionManager this Lane, or any lane it leads into intersects, or
     * <code>Double.MAX_VALUE</code> if no such IntersectionManager exists
     */
    public double distanceToFirstIntersection() {
        if (intersectionManagers.isEmpty()) {
            if (lane.hasNextLane()) {
                return lane.getLength()
                        + lane.getNextLane().getLaneIM().distanceToFirstIntersection();
            }
            return Double.MAX_VALUE;
        }
        // Otherwise, it's the distance from the start of the Lane to the entry
        // point of the first IntersectionManager
        IntersectionManager firstIM
                = intersectionManagers.get(intersectionManagers.firstKey());
        Point2D entry = firstIM.getIntersection().getEntryPoint(lane);
        if (entry == null) {
            return 0; // The Lane starts out in the intersection.
        }
        // Otherwise just return the distance from the start of this Lane to
        // the place it enters the first intersection
        return lane.getStartPoint().distance(entry);
    }

    /**
     * Find the next Lane, including this one, that enters an intersection at
     * any point.
     *
     * @return the next Lane, following the chain of Lanes in which this Lane
     * is, that enters an intersection, at any point
     */
    public Lane laneToFirstIntersection() {
        // If there aren't any more in this lane
        if (intersectionManagers.isEmpty()) {
            // Check the next Lane
            if (lane.hasNextLane()) {
                // Pass the buck to the next Lane after this one
                return lane.getNextLane().getLaneIM().laneToFirstIntersection();
            }
            // Otherwise, there are none.
            return null;
        }
        // Otherwise, it is this one.
        return lane;
    }

    /**
     * Get the last IntersectionManager that this Lane, or any Lane that leads
     * into it enters. Recursively searches through all previous Lanes.
     *
     * @return the last IntersectionManager this Lane, or any Lane that leads
     * into it enters.
     */
    public IntersectionManager lastIntersectionManager() {
        if (intersectionManagers.isEmpty()) {
            if (lane.hasPrevLane()) {
                return lane.getPrevLane().getLaneIM().lastIntersectionManager();
            }
            return null;
        }
        return intersectionManagers.get(intersectionManagers.lastKey());
    }

    /**
     * Get the distance from the end of this Lane to the last
     * IntersectionManager that this Lane, or any Lane that leads into it
     * entered. Recursively searches through all previous Lanes. Returns
     * <code>Double.MAX_VALUE</code> if no such IntersectionManager exists.
     *
     * @return the distance from the end of this Lane to the last
     * IntersectionManager this Lane, or any lane that leads into it entered, or
     * <code>Double.MAX_VALUE</code> if no such IntersectionManager exists
     */
    public double remainingDistanceFromLastIntersection() {
        if (intersectionManagers.isEmpty()) {
            if (lane.hasPrevLane()) {
                return lane.getLength()
                        + lane.getPrevLane().getLaneIM().
                                remainingDistanceFromLastIntersection();
            } else {
                return Double.MAX_VALUE;
            }
        } else {
            return (1 - intersectionManagers.lastKey()) * lane.getLength();
        }
    }

    // given a point -> next im
    /**
     * Find the next IntersectionManager a vehicle at the given position will
     * encounter. These are indexed based on how far along the lane the vehicle
     * is, from 0 (at the start) to 1 (at the end).
     *
     * @param p the location of the hypothetical vehicle
     * @return the next IntersectionManager the vehicle will encounter, or
     * <code>null</code> if none
     */
    public IntersectionManager nextIntersectionManager(Point2D p) {
        // First find how far along the point is.
        double index = lane.normalizedDistanceAlongLane(p);
        SortedMap<Double, IntersectionManager> remaining
                = intersectionManagers.tailMap(index);
        // If nothing left, then no more IntersectionManagers
        if (remaining.isEmpty()) {
            if (lane.hasNextLane()) {
                return lane.getNextLane().getLaneIM().firstIntersectionManager();
            } else {
                return null;
            }
        } else {
            return remaining.get(remaining.firstKey());
        }
    }

    /**
     * Find the distance to the next IntersectionManager a vehicle at the given
     * position will encounter. First projects the point onto the Lane.
     *
     * @param p the current location of the vehicle
     * @return the distance along the Lane from the point on the Lane nearest to
     * the given point to the next IntersectionManager a vehicle at the given
     * point will encounter; if there is no next intersection, return
     * Double.MAX_VALUE
     */
    public double distanceToNextIntersection(Point2D p) {
        // First determine how far along the Lane we are
        double index = lane.normalizedDistanceAlongLane(p);
        // Now find all IntersectionManagers that are after this point (remember
        // they are indexed by exit point)
        SortedMap<Double, IntersectionManager> remaining
                = intersectionManagers.tailMap(index);
        // If there aren't any more in this lane
        if (remaining.isEmpty()) {
            // Check the next Lane
            if (lane.hasNextLane()) {
                return ((1 - index) * lane.getLength())
                        + lane.getNextLane().getLaneIM().distanceToFirstIntersection();
            } else {
                // Otherwise, just say it is really really far away
                return Double.MAX_VALUE;
            }
        } else {
            // Otherwise, we need to figure out where we are and where the current
            // Lane intersects the next intersection.
            IntersectionManager nextIM = remaining.get(remaining.firstKey());
            Point2D entry = nextIM.getIntersection().getEntryPoint(lane);
            // Where does this Lane enter?
            if (entry == null) { // It doesn't! It just exits! That means we're in it!
                return 0.0;
            } else {
                // Otherwise, there is an entry point.  Find out how far along it is in
                // the Lane
                double entryFraction = lane.normalizedDistanceAlongLane(entry);
                // Now, we want to return 0 if we are past the entry point, or the
                // distance to the entry point otherwise
                return Math.max(0.0, (entryFraction - index) * lane.getLength());
            }
        }
    }

    /**
     * Find the next Lane, including this one, that will enter an intersection,
     * starting at the point in this Lane nearest the provided point.
     *
     * @param p the current location of the vehicle
     * @return the next Lane, following the chain of Lanes in which this Lane
     * is, that will enter an intersection, starting at the point in this Lane
     * nearest the provided point
     */
    public Lane laneToNextIntersection(Point2D p) {
        // First determine how far along the Lane we are
        double index = lane.normalizedDistanceAlongLane(p);
        // Now find all IntersectionManagers that are after this point (remember
        // they are indexed by exit point)
        SortedMap<Double, IntersectionManager> remaining
                = intersectionManagers.tailMap(index);
        // If there aren't any more in this lane
        if (remaining.isEmpty()) {
            // Check the next Lane
            if (lane.hasNextLane()) {
                // Pass the buck to the next Lane after this one
                return lane.getNextLane().getLaneIM().laneToFirstIntersection();
            }
            // Otherwise, there are none.
            return null;
        }
        // Otherwise, it is this one.
        return lane;
    }

    // given a point -> prev im
    /**
     * Find the distance from a point, projected onto the Lane, to the previous
     * intersection that a vehicle at that position on the Lane would have
     * encountered.
     *
     * @param p the current location of the vehicle
     * @return the distance from a point, projected onto the Lane, to the
     * previous intersection that a vehicle at that position on the Lane would
     * have encountered
     */
    public double distanceFromPrevIntersection(Point2D p) {
        // First determine how far along the Lane we are
        double index = lane.normalizedDistanceAlongLane(p);
        // Now find all IntersectionManagers that are before this point (remember
        // they are indexed by exit point)
        SortedMap<Double, IntersectionManager> preceding
                = intersectionManagers.headMap(index);
        // If there aren't any in this lane
        if (preceding.isEmpty()) {
            // Check the previous Lane
            if (lane.hasPrevLane()) {
                return (index * lane.getLength())
                        + lane.getNextLane().getLaneIM().
                                remainingDistanceFromLastIntersection();
            }
            // Otherwise, just say it is really really far away
            return Double.MAX_VALUE;
        }
        // preceding.lastKey() is the relative distance to the exit point of the
        // last Intersection in the Lane before our position, so we subtract that
        // from our current relative position (index) to get the total relative
        // distance. Then, multiply that by length to get an absolute distance.
        // This can't be negative because the last key must be before index
        // since we did a headMap.
        return (index - preceding.lastKey()) * lane.getLength();
    }

    // given an im
    /**
     * Get the IntersectionManager that this Lane, or any Lane it leads into
     * enters, after the given IntersectionManager.
     *
     * @param im the IntersectionManager to which we would like the successor
     * @return the IntersectionManager that this Lane, or any Lane it leads into
     * enters, after the given IntersectionManager
     */
    public IntersectionManager nextIntersectionManager(IntersectionManager im) {
        // Build the cache if it doesn't exist
        if (memoGetSubsequentIntersectionManager == null) {
            memoGetSubsequentIntersectionManager
                    = new HashMap<IntersectionManager, IntersectionManager>();
            IntersectionManager lastIM = null;
            // Now run through the IntersectionManagers in order and set up
            // the cache
            for (IntersectionManager currIM : intersectionManagers.values()) {
                // Don't include the first one as a value, since it isn't subsequent
                // to anything
                if (lastIM != null) {
                    memoGetSubsequentIntersectionManager.put(lastIM, currIM);
                }
                lastIM = currIM;
            }
            // Link up to the next Lane
            if (lastIM != null && lane.hasNextLane()) {
                memoGetSubsequentIntersectionManager.put(lastIM,
                        lane.getNextLane().
                                getLaneIM().
                                firstIntersectionManager());
            }
        }
        return memoGetSubsequentIntersectionManager.get(im);
    }

    /**
     * Get the distance from the given IntersectionManager to the next one that
     * that this Lane, or any Lane it leads into enters.
     *
     * @param im the IntersectionManager at which to start
     * @return the distance, in meters, departing the given IntersectionManager,
     * to reach the next IntersectionManager
     */
    public double distanceToNextIntersectionManager(IntersectionManager im) {
        // Two cases: either the next intersection is in this Lane, or it is
        // in a Lane connected to this one
        IntersectionManager nextIM = nextIntersectionManager(im);
        if (nextIM == null) {
            // If there's no next intersection, we just return 0 since the
            // behavior isn't well defined
            return 0;
        }
        if (nextIM.getIntersection().isEnteredBy(lane)) {
            // This is the easy case: just find the distance to the next
            // intersection and divide by the speed limit
            return im.getIntersection().getExitPoint(lane).distance(
                    nextIM.getIntersection().getEntryPoint(lane));
        } else {
            // This is more challenging.  We need to keep adding it up the Lanes
            // in between until we find it
            // Start with the distance to the end of this Lane
            double totalDist = remainingDistanceFromLastIntersection();
            Lane currLane = lane.getNextLane();
            // Okay, add up all the lanes until the IM
            while (!nextIM.getIntersection().isEnteredBy(currLane)) {
                totalDist += currLane.getLength();
                currLane = currLane.getNextLane();
            }
            // Now we're at the Lane that actually enters the next IM
            totalDist += currLane.getLaneIM().distanceToFirstIntersection();
            return totalDist;
        }
    }

    /**
     * Get the approximate time from the given IntersectionManager to the next
     * one that that this Lane, or any Lane it leads into enters, based on
     * distances and speed limits.
     *
     * @param im the IntersectionManager at which to start
     * @param maxVelocity the maximum velocity of the vehicle
     * @return the time, in seconds, that it should take once departing the
     * given IntersectionManager, to reach the next IntersectionManager
     */
    public double timeToNextIntersectionManager(IntersectionManager im,
            double maxVelocity) {
        // Two cases: either the next intersection is in this Lane, or it is
        // in a Lane connected to this one
        IntersectionManager nextIM = nextIntersectionManager(im);
        if (nextIM == null) {
            // If there's no next intersection, we just return 0 since the
            // behavior isn't well defined
            return 0;
        }
        if (nextIM.getIntersection().isEnteredBy(lane)) {
            // This is the easy case: just find the distance to the next
            // intersection and divide by the speed limit
            return im.getIntersection().getExitPoint(lane).distance(
                    nextIM.getIntersection().getEntryPoint(lane))
                    / Math.min(lane.getSpeedLimit(), maxVelocity);
        } else {
            // This is more challenging.  We need to keep adding it up the Lanes
            // in between until we find it
            // Start with the distance to the end of this Lane
            double totalTime = remainingDistanceFromLastIntersection()
                    / lane.getSpeedLimit();
            Lane currLane = lane.getNextLane();
            // Okay, add up all the lanes until the IM
            while (!nextIM.getIntersection().isEnteredBy(currLane)) {
                totalTime += currLane.getLength()
                        / Math.min(currLane.getSpeedLimit(), maxVelocity);
                currLane = currLane.getNextLane();
            }
            // Now we're at the Lane that actually enters the next IM
            totalTime += currLane.getLaneIM().distanceToFirstIntersection()
                    / Math.min(currLane.getSpeedLimit(), maxVelocity);
            return totalTime;
        }
    }

    /**
     * Checks if an action may be taken from a specified lane. Assumes Auto
     * vehicle.
     *
     * @param IM Intersection Manager for which to change the lane restrictions.
     * Changes are stored in this LaneIM object.
     * @param turnDir Action a vehicle wishes to take from lane.
     * @return True if turnDir is a valid action from arrivalLane and
     * arrivalLane is an incoming lane for this intersection.
     */
    /*public boolean isValidActionFromLane(IntersectionManager IM, Constants.TurnDirection turnDir) {
        return isValidActionFromLane(IM, turnDir, null);
    }*/
    /**
     * Checks if an action may be taken from a specified lane. Assumes Auto
     * vehicle if vType is null.
     *
     * @param IM Intersection Manager for which to change the lane restrictions.
     * Changes are stored in this LaneIM object.
     * @param turnDir Action a vehicle wishes to take from lane.
     * @param vType Type of vehicle for which the turn restrictions apply.
     * Assumes Auto vehicle if <code>null</code>.
     * @return True if turnDir is a valid action from arrivalLane and
     * arrivalLane is an incoming lane for this intersection.
     */
    public boolean isValidActionFromLane(IntersectionManager IM, Constants.TurnDirection turnDir, SimConfig.VEHICLE_TYPE vType) {
        //Assume Auto unless specified otherwise
        SimConfig.VEHICLE_TYPE actualType = (vType == null ? SimConfig.VEHICLE_TYPE.AUTO : vType);

        Set<Constants.TurnDirection> acts = null;
        if (laneActionsByVehicleType.containsKey(actualType)) {
            acts = laneActionsByVehicleType.get(actualType).get(IM);
        }

        //If empty set, all actions are allowed. If null, invalid lane or no actions are allowed. Otherwise the acts set must contain the direction for this to return true.
        return Collections.EMPTY_SET.equals(acts) || (acts != null && acts.contains(turnDir));
    }

    /**
     * Gets allowed actions from lane if it exists. Assumes Auto vehicle.
     *
     * @param IM Intersection Manager for which to change the lane restrictions.
     * Changes are stored in this LaneIM object.
     * @return Empty set of TurnActions if all actions allowed.
     * <code>null</code> if the lane is not valid or no actions allowed.
     * Otherwise array of TurnActions that are valid.
     */
    /*public Set<Constants.TurnDirection> validActionsFromLane(IntersectionManager IM) {
        return validActionsFromLane(IM, null);
    }*/
    /**
     * Gets allowed actions from lane if it exists. Assumes Auto vehicle if
     * vType is null.
     *
     * @param IM Intersection Manager for which to change the lane restrictions.
     * Changes are stored in this LaneIM object.
     * @param vType Type of vehicle for which the turn restrictions apply.
     * Assumes Auto vehicle if <code>null</code>.
     * @return Empty set of TurnActions if all actions allowed.
     * <code>null</code> if the lane is not valid or no actions allowed.
     * Otherwise array of TurnActions that are valid.
     */
    public Set<Constants.TurnDirection> validActionsFromLane(IntersectionManager IM, SimConfig.VEHICLE_TYPE vType) {
        //Assume Auto unless specified otherwise
        SimConfig.VEHICLE_TYPE actualType = (vType == null ? SimConfig.VEHICLE_TYPE.AUTO : vType);

        if (laneActionsByVehicleType.containsKey(actualType)) {
            return laneActionsByVehicleType.get(actualType).get(IM);
        }
        return null;
    }

    /**
     * Allows specification of valid TurnDirections from lane. Will cause
     * undefined behavior if used after simulation start. Assumes Auto vehicle.
     *
     * @param IM Intersection Manager that manages lane.
     * @param validActions Actions in the form of a Set of TurnDirections to
     * associate with the lane as "valid".
     *
     * @return True if IM was a valid IntersectionManager and the validActions
     * were successfully associated with the lane. False otherwise.
     */
    /*public boolean setValidActionsForLane(IntersectionManager IM, Set<Constants.TurnDirection> validActions) {
        return setValidActionsForLane(IM, validActions, null);
    }*/
    /**
     * Allows specification of valid TurnDirections from lane. Will cause
     * undefined behavior if used after simulation start. Assumes Auto vehicle
     * if vType is null.
     *
     * @param IM Intersection Manager that manages lane.
     * @param validActions Actions in the form of a Set of TurnDirections to
     * associate with the lane as "valid".
     * @param vType Type of vehicle for which the turn restrictions apply.
     *
     * @return True if IM was a valid IntersectionManager and the validActions
     * were successfully associated with the lane. False otherwise.
     */
    public boolean setValidActionsForLane(IntersectionManager IM, Set<Constants.TurnDirection> validActions, SimConfig.VEHICLE_TYPE vType) {
        //Assume Auto unless specified otherwise
        SimConfig.VEHICLE_TYPE actualType = (vType == null ? SimConfig.VEHICLE_TYPE.AUTO : vType);

        if (laneActionsByVehicleType.containsKey(actualType)) {
            if (laneActionsByVehicleType.get(actualType).keySet().contains(IM)) {
                if (validActions == null || validActions.equals(Collections.EMPTY_SET)) {
                    laneActionsByVehicleType.get(actualType).put(IM, validActions);
                } else {
                    laneActionsByVehicleType.get(actualType).put(IM, Collections.unmodifiableSet(EnumSet.copyOf(validActions)));
                }
                IM.resetToBlankExitRoadAndLaneMaps(lane.getContainingRoad());
                return true;
            }
        }

        return false;
    }

    /**
     * Sets map of distance in lane to vehicles in lane.
     *
     * @param vecs Map of distance in lane to vehicles in lane.
     */
    public void setVehiclesInLane(SortedMap<Double, VehicleSimView> vecs) {
        vehiclesInLane = vecs;
        vehiclesCountCache = new TreeMap<Double, Integer>();
    }

    /**
     * Finds number of vehicles between the point (inclusive) at the distance
     * provided along the lane and the LaneIM's lane's entry point at the next
     * intersection (inclusive).
     *
     * @param dist Distance along the lane to begin looking.
     * @return Number of vehicles in the lane between the point given and the
     * intersection.
     */
    public int getVehiclesToNextIntersection(Double dist) {
        //getting only the vehicles that are in the lane at or after the provided point
        SortedMap<Double, VehicleSimView> applicableVehicles = vehiclesInLane.tailMap(dist);
        double startIndex;
        if (!applicableVehicles.isEmpty()) {
            startIndex = applicableVehicles.firstKey();
        } else {
            startIndex = 0.0;
        }

        if (!vehiclesCountCache.containsKey(startIndex)) {
            int count = 0;
            //get attributes about the intersection neccessary for determining whether a vehicle has passed the distance in the lane where it should be counted.
            IntersectionManager nextIM = nextIntersectionManager(lane.getPointAtNormalizedDistance(dist));
            Point2D.Double entryPoint = nextIM.getIntersection().getEntryPoint(lane);
            double maxDistance = lane.distanceAlongLane(entryPoint);
            Double nextKey = vehiclesCountCache.higherKey(startIndex);

            for (double key : applicableVehicles.keySet()) {
                double vecDistInLane = lane.distanceAlongLane(applicableVehicles.get(key).getPosition());
                if (nextKey != null && key == nextKey) {
                    count += vehiclesCountCache.get(nextKey);
                } else if (vecDistInLane < maxDistance) {
                    ++count;
                } else {
                    break;
                }
            }
            vehiclesCountCache.put(startIndex, count);
        }

        return vehiclesCountCache.get(startIndex);
    }

    /**
     * Marks lane as having no outgoing mapping for all TurnDirections by
     * clearing the outgoing mapping for this intersection. Does nothing if the
     * IntersectionManager isn't valid.
     *
     * @param IM Intersection at which the mapping occurs.
     */
    public void disallowAllOutgoingLaneMappings(IntersectionManager IM) {
        if (laneToLaneByAction.containsKey(IM)) {
            laneToLaneByAction.put(IM, null);
            for (Road road : IM.getIntersection().getExitRoads()) {
                IM.getTrackModel().resetCachedRestrictedLanePriorities(lane, road);
            }
        }
    }

    /**
     * Marks lane as having any outgoing mapping for all TurnDirections by
     * clearing the outgoing mapping for this intersection. Does nothing if the
     * IntersectionManager isn't valid.
     *
     * @param IM Intersection at which the mapping occurs.
     */
    public void allowAllOutgoingLaneMappings(IntersectionManager IM) {
        if (laneToLaneByAction.containsKey(IM)) {
            laneToLaneByAction.put(IM, new HashMap<Constants.TurnDirection, Lane>(Constants.TurnDirection.values().length));
            for (Road road : IM.getIntersection().getExitRoads()) {
                IM.getTrackModel().resetCachedRestrictedLanePriorities(lane, road);
            }
        }
    }

    /**
     * Sets the lane which all vehicles must go to if taking the provided
     * turning action (provided they are even allowed to) from this lane at the
     * provided intersection.
     *
     * @param IM Intersection at which the mapping occurs.
     * @param td TurnDirection for which to make the lane-to-lane mapping at the
     * provided intersection.
     * @param ln Outgoing lane in the lane-to-lane mapping.
     */
    public void setExitLaneMapping(IntersectionManager IM, Constants.TurnDirection td, Lane ln) {
        if (laneToLaneByAction.containsKey(IM) && IM.manages(ln)) {
            HashMap<Constants.TurnDirection, Lane> innerMap = laneToLaneByAction.get(IM);
            if (innerMap == null) {
                //resets maps for this lane at this intersection if mapping was previously disabled
                laneToLaneByAction.put(IM, new HashMap<Constants.TurnDirection, Lane>(Constants.TurnDirection.values().length));
                innerMap = laneToLaneByAction.get(IM);
                IM.getTrackModel().resetCachedRestrictedLanePriorities(lane, ln.getContainingRoad());
            }

            innerMap.put(td, ln);
        }
    }

    /**
     * Gets exit lane based on Intersection Manager and TurnDirection. This is
     * useful with virtual intersection architecture when "disabling" lanes.
     *
     * @param IM IntersectionManager at which the mapping occurs.
     * @param td Direction of turn/action at the intersection.
     * @return Lane designated as the exit lane at the intersection associated
     * with the provided IntersectionManager for this LaneIM's Lane. Null if no
     * lane is explicitly mapped for the action. Use in combination with
     * isValidActionFromLane to determine if the action is allowed from the
     * lane. If isValidActionFromLane returns true and this returns null, then
     * the action is allowed but there is not a specifically mapped lane.
     */
    public Lane getMappedExitLane(IntersectionManager IM, Constants.TurnDirection td) {
        HashMap<Constants.TurnDirection, Lane> innerMap = laneToLaneByAction.get(IM);
        if (innerMap != null) {
            return innerMap.get(td);
        }
        return null;
    }

    /**
     * Gets TurnDirections that have been mapped based on Intersection Manager.
     * This is useful with virtual intersection architecture when "disabling"
     * lanes.
     *
     * @param IM IntersectionManager at which the mapping occurs.
     * @return Set of TurnDirections used to map lanes to their exit lane at a
     * particular intersection. EMPTY_SET if all lanes allowed.
     * <code>Null</code> if invalid IntersectionManager or no mapped outgoing
     * Lanes.
     */
    //todo, should just fill with Constants.ACTABLE_TURN_DIRECTIONS if all actions are allowed, but a bunch of logic may need to be checked to ensure this is okay because of the way I did this the first time (before ACTABLE_TURN_DIRECTIONS existed, I think).
    public Set<Constants.TurnDirection> getMappedTurnDirectionsForAllVehicleTypes(IntersectionManager IM) {
        HashMap<Constants.TurnDirection, Lane> innerMap = laneToLaneByAction.get(IM);
        if (innerMap != null) {
            Set<Constants.TurnDirection> dirs = innerMap.keySet();
            if (dirs != null) {
                if (dirs.isEmpty()) {
                    return Collections.EMPTY_SET;
                } else {
                    return Collections.unmodifiableSet(dirs);
                }
            }
        }
        return null;
    }

    public void setWithTrafficTurnOnRedAllowed(boolean allowed) {
        allowWithTrafficTurnsOnRed = allowed;
    }

    /**
     * Returns if right turns on red are permitted for any vehicle type. To
     * check if a right turn on red may be performed for a specific vehicle, you
     * must also check if that vehicle is allowed to turn right on this LaneIM's
     * associated lane.
     *
     * @param IM
     * @return whether right turns are permitted on this LaneIM's lane for any
     * vehicle type.
     */
    public boolean getWithTrafficTurnOnRedAllowed(IntersectionManager IM) {
        Set<Constants.TurnDirection> tds = getMappedTurnDirectionsForAllVehicleTypes(IM);

        //firstly does the lane allow it, secondly does the lane support turning with traffic at all?
        return allowWithTrafficTurnsOnRed && tds != null && (tds.equals(Collections.EMPTY_SET) || tds.contains(Constants.WITH_TRAFFIC_TURN_DIRECTION));
    }

    /**
     * This function is a bit of a hack around the way the intersection and
     * vehicle interact. This is used so when a vehicle is within transmission
     * distance it will call this function so that the intersection "sees" the
     * vehicle arriving and logs it for purposes of arrival analysis (but not
     * direct signal actuation)
     *
     * @param im
     * @param vehicle
     * @return true if successful, false otherwise
     */
    public boolean logArrivalOnLane(IntersectionManager im, AutoVehicleDriverView vehicle) {
        if (im != null) {
            double time = im.getCurrentTime();
            if (imToArrivalsMap.get(im) == null) {
                imToArrivalsMap.put(im, new HashMap<AutoVehicleDriverView, Double>());
            }
            imToArrivalsMap.get(im).put(vehicle, time);
            timesLeftForWhichToLogTurnDirections.add(time);

            /*if (timesLeftForWhichToLogTurnDirections.peek() < time - timeToKeepTDEntry) {
                throw new RuntimeException("Vehicle that arrived at the intersection and was logged as arriving at " + timesLeftForWhichToLogTurnDirections.peek() + " wasn't logged as leaving within the allowed time span.");
            }*/
            return true;
        }
        return false;
    }

    /**
     * This function is a bit of a hack around the way the intersection and
     * vehicle interact. This is used so when a vehicle exits an intersection it
     * will call this function so that the intersection "sees" the vehicle's
     * turn direction and logs it for purposes of arrival analysis
     *
     * @param im
     * @param vehicle
     * @param exitLane
     * @return true if successful, false otherwise
     */
    public boolean logTurnDirectionFromExitingVehicle(IntersectionManager im, AutoVehicleDriverView vehicle, Lane exitLane) {
        if (im != null) {
            double time;

            if (imToArrivalsMap.get(im) == null || imToArrivalsMap.get(im).get(vehicle) == null) {
                return false;
            } else {
                time = imToArrivalsMap.get(im).remove(vehicle);
            }

            if (imToTDArrivalsMap.get(im) == null) {
                imToTDArrivalsMap.put(im, new LinkedList<LimitedPairImplementation<Double, Constants.TurnDirection>>());
            }
            Constants.TurnDirection td = Util.getTurnDirectionFromArrivalAndDepartureLanesForCardinalDirections(lane, exitLane, im);
            imToTDArrivalsMap.get(im).add(new LimitedPairImplementation<Double, Constants.TurnDirection>(time, td));
            timesLeftForWhichToLogTurnDirections.remove(time);
            if (time >= im.getCurrentTime()-timeToKeepTDEntry) {
                if (countForTDsInLane.get(im) == null) {
                    countForTDsInLane.put(im, new HashMap<Constants.TurnDirection, Integer>());
                }

                if (countForTDsInLane.get(im).get(td) == null) {
                    countForTDsInLane.get(im).put(td, 1);
                } else {
                    countForTDsInLane.get(im).put(td, countForTDsInLane.get(im).get(td) + 1); //lazy logging, old entries get pulled back out on checks for the number
                }
            }

            return true;
        }
        return false;
    }

    /**
     *
     * @param im intersection manager for the intersection we wish to track
     * @param tds turn directions to check
     * @return the number of vehicles performing any of the provided turn
     * movements at the intersection for the given im. 0 if the im is invalid.
     */
    public int getNumberOfArrivalsOnLaneForTurnDirections(IntersectionManager im, Set<Constants.TurnDirection> tds) {
        if (imToTDArrivalsMap.get(im) == null) {
            return 0;
        }

        while (!imToTDArrivalsMap.get(im).isEmpty()) {
            if (imToTDArrivalsMap.get(im).peek().getKey() < im.getCurrentTime() - timeToKeepTDEntry) {
                LimitedPairImplementation<Double, Constants.TurnDirection> toRemove = imToTDArrivalsMap.get(im).poll();
                countForTDsInLane.get(im).put(toRemove.getValue(), countForTDsInLane.get(im).get(toRemove.getValue()) - 1);
            } else {
                break;
            }
        }

        int count = 0;
        for (Constants.TurnDirection td : tds) {
            if (countForTDsInLane.get(im).get(td) != null) {
                count += countForTDsInLane.get(im).get(td);
            }
        }

        return count;
    }
}
