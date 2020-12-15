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
package aim4.im;

import aim4.config.Constants;
import aim4.config.Resources;
import aim4.config.SimConfig;
import aim4.config.ringbarrier.RingAndBarrier;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Collections;
import java.util.List;
import aim4.map.Road;
import aim4.map.lane.Lane;
import aim4.util.Registry;
import aim4.util.Util;
import aim4.vehicle.VehicleSimView;
import java.awt.Point;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;

/**
 * An agent to manage an intersection. This is an abstract class that sets up
 * the properties of the intersection when it is created.
 */
public class IntersectionManager {

    /////////////////////////////////
    // PRIVATE FIELDS
    /////////////////////////////////
    /**
     * The ID number of this intersection manager.
     */
    protected int id;

    /**
     * the current time of the intersection manager
     */
    protected double currentTime;

    /**
     * The intersection managed by this intersection manager.
     */
    private Intersection intersection;
    /**
     * The path model of the intersection.
     */
    private TrackModel trackModel;
    /**
     * Double layered map of incoming roads to outgoing roads in terms of
     * allowed lane actions. (For instance, one road may not be reachable from
     * another depending on lane restrictions.)
     */
    private HashMap<Road, HashMap<Road, Boolean>> roadToRoad;

    /**
     * Map of exit lane->set of entry lanes which can be used to reach that exit
     * lane based on allowed turning actions
     */
    private HashMap<Lane, Set<Lane>> exitLanesToEntryLanesByMappedTurningDirections;

    /**
     * Ring and barrier (or null if none) for this intersection
     */
    private RingAndBarrier ringAndBarrier;

    private HashMap<Lane, Double> maxAllowedFutureReservationTimeOnLanes;

    private Lane laneWithShortestTimeToIntersection;

    /////////////////////////////////
    // CLASS CONSTRUCTORS
    /////////////////////////////////
    /**
     * Create an intersection manager.
     *
     * @param intersection an intersection
     * @param trackModel a path model of the intersection
     * @param currentTime the current time
     * @param imRegistry an intersection manager registry
     * @param ringAndBarrier ring and barrier for intersection
     */
    public IntersectionManager(Intersection intersection,
            TrackModel trackModel,
            double currentTime,
            Registry<IntersectionManager> imRegistry,
            RingAndBarrier ringAndBarrier) {
        assert (trackModel.getIntersection() == intersection);
        this.intersection = intersection;
        this.trackModel = trackModel;
        this.currentTime = currentTime;
        this.id = imRegistry.register(this);
        this.ringAndBarrier = ringAndBarrier;

        exitLanesToEntryLanesByMappedTurningDirections = null;
        laneWithShortestTimeToIntersection = null;

        //todo, this is an indirect leak of the object reference in the constructor and isn't best practice. Should be refactored to avoid this.
        // Register the intersection manager with the lanes
        registerWithLanes();

        //set up empty road to road mapping:
        roadToRoad = new HashMap<Road, HashMap<Road, Boolean>>();
        for (Road rd : intersection.getEntryRoads()) {
            roadToRoad.put(rd, null);
            resetToBlankExitRoadAndLaneMapsPrivate(rd);
        }

        if (imRegistry.getValues().size() > 1) {
            throw new RuntimeException("Modifications were made to the simulator in IntersectionManager for realistic traffic conditions which assume only a single intersection. These must be modified before more intersections can be considered.");
        }
        //this assumes the distance to the intersection is spawnpoint to intersection, which is incorrect if there are multiple intersections
        for (Lane ln : this.intersection.getEntryLanes()) {
            if (laneWithShortestTimeToIntersection == null) {
                laneWithShortestTimeToIntersection = ln;
            } else if (laneWithShortestTimeToIntersection.getStartPoint().distance(this.intersection.getEntryPoint(laneWithShortestTimeToIntersection)) / laneWithShortestTimeToIntersection.getSpeedLimit() < ln.getStartPoint().distance(this.intersection.getEntryPoint(ln)) / ln.getSpeedLimit()) {
                laneWithShortestTimeToIntersection = ln;
            }
        }

        maxAllowedFutureReservationTimeOnLanes = new HashMap<Lane, Double>();
    }

    /**
     * Create an intersection manager.
     *
     * @param intersection an intersection
     * @param trackModel a path model of the intersection
     * @param currentTime the current time
     * @param imRegistry an intersection manager registry
     */
    public IntersectionManager(Intersection intersection,
            TrackModel trackModel,
            double currentTime,
            Registry<IntersectionManager> imRegistry) {
        this(intersection, trackModel, currentTime, imRegistry, null);
    }

    /**
     * Register this IntersectionManager with each of the Lanes that it manages.
     */
    private void registerWithLanes() {
        for (Lane lane : intersection.getLanes()) {
            lane.getLaneIM().registerIntersectionManager(this);
        }
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    /**
     * Take any actions for a certain period of time.
     *
     * @param timeStep the size of the time step to simulate, in seconds
     */
    public void act(double timeStep) {
        currentTime += timeStep;
    }

    /**
     * Get the unique ID number of this IntersectionManager.
     *
     * @return the ID number of this IntersectionManager
     */
    public int getId() {
        return id;
    }

    /**
     * Get the current time.
     *
     * @return the current time.
     */
    public double getCurrentTime() {
        return currentTime;
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    // intersection
    /**
     * Get the intersection managed by this intersection manager.
     *
     * @return the intersection managed by this intersection manager
     */
    public Intersection getIntersection() {
        return intersection;
    }

    /**
     * Get the track model.
     *
     * @return the track model
     */
    public TrackModel getTrackModel() {
        return trackModel;
    }

    /**
     * Whether or not this IntersectionManager manages the given Road.
     *
     * @param r the Road
     * @return whether this IntersectionManager manages the given Road
     */
    public boolean manages(Road r) {
        return intersection.getLanes().contains(r.getIndexLane());
    }

    /**
     * Whether or not this IntersectionManager manages the given Lane.
     *
     * @param l the Lane
     * @return whether this IntersectionManager manages the given Lane
     */
    public boolean manages(Lane l) {
        return intersection.getLanes().contains(l);
    }

    /**
     * Determine whether the given Vehicle is currently entirely contained
     * within the Area governed by this IntersectionManager.
     *
     * @param vehicle the Vehicle
     * @return whether the Vehicle is currently entirely contained within the
     * Area governed by this IntersectionManager
     */
    public boolean contains(VehicleSimView vehicle) {
        // Get all corners of the vehicle and make sure they are inside the
        // intersection.
        for (Point2D corner : vehicle.getCornerPoints()) {
            if (!intersection.getArea().contains(corner)) {
                return false;
            }
        }
        // If all corners are inside, the whole thing is considered inside.
        return true;
    }

    /**
     * Determine whether the given Rectangle intersects the Area governed by
     * this IntersectionManager.
     *
     * @param rectangle the Rectangle
     * @return whether the Rectangle intersects the Area governed by this
     * IntersectionManager
     */
    public boolean intersects(Rectangle2D rectangle) {
        // Just call the Area method, so we don't have to clone the area.
        // Make sure not to use "intersect" which is destructive.
        return intersection.getArea().intersects(rectangle);
    }

    /**
     * Given an arrival Lane and a departure Road, get an ordered List of Lanes
     * that represents the Lanes from highest to lowest priority based on
     * distance from the arrival Lane.
     *
     * @param arrivalLane the Lane in which the vehicle is arriving
     * @param departure the Road by which the vehicle is departing
     * @return the ordered List of Lanes, by priority, into which the vehicle
     * should try to turn
     */
    public List<Lane> getSortedDepartureLanes(Lane arrivalLane, Road departure) {
        //Constants.TurnDirection td = intersection.calcTurnDirection(arrivalLane, departure.getIndexLane());
        return trackModel.getRestrictedSortedDepartureLanes(arrivalLane, departure);

        //return trackModel.getSortedDepartureLanes(arrivalLane, departure);
    }

    /**
     * Get the distance from the entry of the given Road, to the departure of
     * the other given Road.
     *
     * @param arrival the arrival Road
     * @param departure the departure Road
     * @return the distance from the entry of the arrival Road to the exit of
     * the departure Road
     */
    public double traversalDistance(Road arrival, Road departure) {
        return trackModel.traversalDistance(arrival, departure);
    }

    /**
     * Get the distance from the entry of the given Lane, to the departure of
     * the other given Lane, if traveling along segments through their point of
     * intersection.
     *
     * @param arrival the arrival Lane
     * @param departure the departure Lane
     * @return the distance from the entry of the arrival Lane to the exit of
     * the departure Lane through their intersection
     */
    public double traversalDistance(Lane arrival, Lane departure) {
        return trackModel.traversalDistance(arrival, departure);
    }

    /**
     * Get the distance from the entry of the Lane with the first given ID, to
     * the departure of the Lane with the other given ID, if traveling along
     * segments through their point of intersection.
     *
     * @param arrivalID the ID number of the arrival Lane
     * @param departureID the ID number of the departure Lane
     * @return the distance from the entry of the arrival Lane to the exit of
     * the departure Lane through their intersection
     */
    public double traversalDistance(int arrivalID, int departureID) {
        return trackModel.traversalDistance(arrivalID, departureID);
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    // statistics
    /**
     * Print the collected data to a file
     *
     * @param outFileName the name of the file to which the data are outputted.
     */
    public void printData(String outFileName) {
    }

    /**
     *
     * @return Ring and barrier for this manager. Null if none.
     */
    public RingAndBarrier getRingAndBarrier() {
        return ringAndBarrier;
    }

    /////////////////////////////////
    // DEBUG
    /////////////////////////////////
    /**
     * Check whether this intersection manager's time is current.
     *
     * @param currentTime the current time
     */
    public void checkCurrentTime(double currentTime) {
        assert Util.isDoubleEqual(currentTime, this.currentTime);
    }

    /**
     * Get any shapes to display for debugging purposes.
     *
     * @return any shapes to display for debugging purposes
     */
    public List<? extends Shape> getDebugShapes() {
        return Collections.emptyList(); // Nothing by default
    }

    /**
     * Determines if outgoing road from intersection can be reached from
     * incoming road to intersection based on turn restrictions. Behavior is
     * only defined on intersections with up to 2 sets of roads.
     *
     * @param inRoad Incoming road to intersection.
     * @param outRoad Outgoing road from intersection.
     * @param vType Vehicle type for which turn restrictions should be checked.
     * @return True if a lane exists in inRoad that will allow a turn to
     * outRoad. False if no such lane exists, inRoad is invalid, or outRoad is
     * invalid.
     */
    public boolean canTakeRoadFromRoad(Road inRoad, Road outRoad, SimConfig.VEHICLE_TYPE vType) {
        HashMap<Road, Boolean> innerMap = roadToRoad.get(inRoad);
        if (innerMap != null && innerMap.containsKey(outRoad)) {
            Boolean validPath = innerMap.get(outRoad);
            //check if reachability has already been calculated. If so, return it. If not, calculate it, store it, and return it.
            if (validPath == null) {
                //todo: I may need to write a "road" version of this function rather than using the index lanes...
                Constants.TurnDirection td = intersection.calcTurnDirection(inRoad.getIndexLane(), outRoad.getIndexLane());
                innerMap.put(outRoad, false);
                for (Lane ln : inRoad.getLanes()) {
                    if (ln.getLaneIM().isValidActionFromLane(this, td, vType)) {
                        innerMap.put(outRoad, true);
                        return true;
                    }
                }
                return false;
            } else {
                return validPath;
            }
        } else {
            return false;
        }
    }

    /**
     * Resets the inner map of the roadToRoad map so that no exiting roads are
     * considered to be known to be reachable from the provided entering road.
     * Similarly clears maps of lanes reachable by other lanes. Does nothing if
     * the provided entering road is not valid.
     *
     * @param entryRoad Road entering the intersection for which the known
     * exiting possibilities need to be reset.
     */
    private void resetToBlankExitRoadAndLaneMapsPrivate(Road entryRoad) {
        if (roadToRoad.containsKey(entryRoad)) {
            exitLanesToEntryLanesByMappedTurningDirections = null;
            HashMap<Road, Boolean> innerMap = new HashMap<Road, Boolean>();
            for (Road innerRoad : intersection.getExitRoads()) {
                innerMap.put(innerRoad, null);
            }
            roadToRoad.put(entryRoad, innerMap);
        }
    }

    /**
     * Resets the inner map of the roadToRoad map so that no exiting roads are
     * considered to be known to be reachable from the provided entering road.
     * Similarly clears maps of lanes reachable by other lanes. Does nothing if
     * the provided entering road is not valid.
     *
     * @param entryRoad Road entering the intersection for which the known
     * exiting possibilities need to be reset.
     */
    public void resetToBlankExitRoadAndLaneMaps(Road entryRoad) {
        resetToBlankExitRoadAndLaneMapsPrivate(entryRoad);
    }

    /**
     * Returns a set of lanes which have some turning action mapping to the
     * provided lane as a destination lane. Lazily populates the map in order to
     * provide this data. As a side effect, also checks for explicit duplicate
     * mappings of turn actions and prints a warning if any are found.
     *
     * @param destinationQueryLane
     * @return a set of lanes which have some turning action mapping to the
     * provided lane as a destination lane. Null if there are no outgoing
     * mappings to the provided lane.
     */
    public Set<Lane> getLanesWhichHaveAnExitLaneMappingByAnyActionAndAnyVehicleTypeToLane(Lane destinationQueryLane) {
        if (exitLanesToEntryLanesByMappedTurningDirections == null) {
            exitLanesToEntryLanesByMappedTurningDirections = new HashMap<Lane, Set<Lane>>();
            //lanes can only be this way, that is a turn direction can end in any lane in the road to which that turn direction maps, when no one-to-one mapping has been set and the lane hasn't specifically been closed. There is no other way to achieve an N-to-1 mapping as of when this was written.
            Set<Lane> lanesWithOneToNMappings = new HashSet<Lane>();

            //used for checking same turn direction explicit mappings ending up at one destination lane
            //departing lane, turn direction, arriving lane
            HashMap<Lane, HashMap<Constants.TurnDirection, Lane>> dupCheckMap = new HashMap<Lane, HashMap<Constants.TurnDirection, Lane>>();

            for (Road rd : intersection.getRoads()) {
                for (Lane arrivalLane : rd.getLanes()) {
                    Set<Constants.TurnDirection> tds = arrivalLane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(this);

                    if (tds == null) {
                        continue; //no tds mapped or this IM was invalid somehow 
                    } else if (tds.equals(Collections.EMPTY_SET)) {
                        lanesWithOneToNMappings.add(arrivalLane);
                    } else {
                        for (Constants.TurnDirection td : tds) {
                            Lane destLane = arrivalLane.getLaneIM().getMappedExitLane(this, td);
                            if (!exitLanesToEntryLanesByMappedTurningDirections.containsKey(destLane)) {
                                exitLanesToEntryLanesByMappedTurningDirections.put(destLane, new HashSet<Lane>());
                            }
                            exitLanesToEntryLanesByMappedTurningDirections.get(destLane).add(arrivalLane);

                            //check if multiple of the same turn direction have been mapped to one lane
                            if (dupCheckMap.get(destLane) == null) {
                                dupCheckMap.put(destLane, new HashMap<Constants.TurnDirection, Lane>());
                            }
                            if (dupCheckMap.get(destLane).get(td) == null) {
                                dupCheckMap.get(destLane).put(td, arrivalLane);
                            } else {
                                Lane previousLaneInConflict = dupCheckMap.get(destLane).get(td);
                                System.err.println("WARNING: Multiple lanes ([ID: " + previousLaneInConflict.getId() + ", Index in road: " + previousLaneInConflict.getIndexInRoad() + ", Road Name: " + previousLaneInConflict.getContainingRoad().getName() + "],"
                                        + " [ID: " + arrivalLane.getId() + ", Index in road: " + arrivalLane.getIndexInRoad() + ", Road Name: " + arrivalLane.getContainingRoad().getName() + "]) both map to lane "
                                        + "[ID: " + destLane.getId() + ", Index in road: " + destLane.getIndexInRoad() + ", Road Name: " + destLane.getContainingRoad().getName() + "] with the same turning action of " + td.name() + ". Please check your intersection architecture specification to ensure this wasn't an error. Note, this error is not shown when all mappings are implicit.");
                            }
                        }
                    }

                }
            }

            if (!lanesWithOneToNMappings.isEmpty()) {
                //one final pass for the n-to-1 mappings
                for (Road rd : intersection.getRoads()) {
                    for (Lane destinationLane : rd.getLanes()) {
                        for (Lane arrivalLane : lanesWithOneToNMappings) {
                            //prevent uturn mapping and also don't indicate lanes map as incoming lanes to lanes within the same road except in the case where a lane proceeds straight to itself
                            if (destinationLane.getContainingRoad().getDual() == arrivalLane.getContainingRoad()) {
                                continue;
                            } else if (destinationLane.getContainingRoad() == arrivalLane.getContainingRoad() && destinationLane != arrivalLane) {
                                continue;
                            }

                            if (!exitLanesToEntryLanesByMappedTurningDirections.containsKey(destinationLane)) {
                                exitLanesToEntryLanesByMappedTurningDirections.put(destinationLane, new HashSet<Lane>());
                            }
                            exitLanesToEntryLanesByMappedTurningDirections.get(destinationLane).add(arrivalLane);
                        }
                    }
                }
            }
        }

        if (exitLanesToEntryLanesByMappedTurningDirections.get(destinationQueryLane) != null) {
            return Collections.unmodifiableSet(exitLanesToEntryLanesByMappedTurningDirections.get(destinationQueryLane));
        } else {
            return null;
        }
    }

    /**
     * The max allowed reserve time is a liiiiitle bit of a misnomer in that an
     * edge condition in the approxNphases request handler has to deal with AVs
     * spawning before HVs
     *
     * @param lane
     * @return The minimum of (the aheadReservationTime allowed for any road
     * entering the intersection, the time it would take for a vehicle on ANY
     * lane (excluding the provided one) following the speed limit to arrive at
     * the intersection from the lane start)
     */
    public double getMaxAllowedFutureReservationTimeOnLane(Lane lane) {
        if (manages(lane)) {
            if (Resources.map.getIntersectionManagers().size() != 1) {
                throw new RuntimeException("getMaxAllowedFutureReservationTimeOnLane doesn't allow multiple intersections at the moment. This would require that we model when a human vehicle can actually be seen (which isn't done at the moment, either we assume it is within sensor range provided it is spawned or we assume crossing any green trajectory at all is off limits).");
            } else if (!SimConfig.FULLY_OBSERVING) {
                throw new RuntimeException("getMaxAllowedFutureReservationTimeOnLane assumes that a human vehicle can be detected as soon as it is spawned. Vehicle detection ranges for environments that aren't fully observable have not been implemented.");
                //if you modify this, see the TODO note in the loop below
            }

            if (maxAllowedFutureReservationTimeOnLanes.get(lane) == null) {
                if (lane == laneWithShortestTimeToIntersection) {
                    double maxAllowedReserveTime = Double.POSITIVE_INFINITY;
                    //check other lanes to see what the fastest another lane could reach the intersection is
                    for (Road road : intersection.getEntryRoads()) {
                        for (Lane incomingLane : road.getLanes()) {
                            if (incomingLane == lane) {
                                continue;
                            } else {
                                maxAllowedReserveTime = Math.min(lane.getContainingRoad().getAheadReservationTimeAllowedForRoadWithoutIntersectionSpecificAdjustment(), Math.min(maxAllowedReserveTime, incomingLane.getStartPoint().distance(intersection.getEntryPoint(incomingLane)) / incomingLane.getSpeedLimit()));
                            }
                        }
                    }
                    maxAllowedFutureReservationTimeOnLanes.put(lane, maxAllowedReserveTime);
                } else {
                    maxAllowedFutureReservationTimeOnLanes.put(lane, Math.min(lane.getContainingRoad().getAheadReservationTimeAllowedForRoadWithoutIntersectionSpecificAdjustment(), laneWithShortestTimeToIntersection.getStartPoint().distance(intersection.getEntryPoint(laneWithShortestTimeToIntersection)) / laneWithShortestTimeToIntersection.getSpeedLimit()));
                }
            }
            return maxAllowedFutureReservationTimeOnLanes.get(lane);
        } else {
            throw new RuntimeException("This IM doesn't manage the provided lane, so no allowable future reservation time can be provided.");
        }
    }

}
