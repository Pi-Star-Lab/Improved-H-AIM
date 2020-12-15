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
package aim4.map;

import aim4.config.Constants;
import aim4.config.SimConfig;
import aim4.im.IntersectionManager;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import aim4.map.lane.Lane;
import aim4.util.Util;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * A group of lanes with a name.
 */
public class Road {

    /////////////////////////////////
    // PRIVATE FIELDS
    /////////////////////////////////
    /**
     * The name of this road.
     */
    private String name;
    /**
     * The lanes that make up this road, from left to right.
     */
    private List<Lane> lanes;
    /**
     * The Road that follows this one in the opposite direction.
     */
    private Road dual;
    /**
     * The Layout of which the Road is a part.
     */
    private BasicMap map;
    /*
    * The time ahead of arrival a vehicle can make a reservation. May be overidden by a lane specific value (not implemented as of 11/13/19).
     */
    private Double maxAheadReservationTimeForRoad;
    /*
    * The speed limit of the road if ALL lanes share the same speed
     */
    private Double roadSpeedLimit;
    /*
    * Both of these memoize lookups for which lanes can safely turn right on red for incoming lanes
     */
    private HashMap<IntersectionManager, Lane> turnOnRedIncomingMemoization;
    private HashMap<IntersectionManager, HashMap<Lane, Integer>> incomingLanesWhichMightSupportTurningOnRed;

    /*
    * Both of these memoize lookups for which lanes can safely be turned onto for right on red for outgoing lanes
     */
    private HashMap<IntersectionManager, Lane> turnOnRedOutgoingMemoization;
    private HashMap<IntersectionManager, HashMap<Lane, Integer>> outgoingLanesWhichMightSupportTurningOnRed;

    private HashMap<Lane, Integer> relativeLaneIndexInRoad;

    private int index;
    private static int roadCount = 0;

    public int getIndex() {
        return index;
    }

    /////////////////////////////////
    // CLASS CONSTRUCTORS
    /////////////////////////////////
    /**
     * Create a new Road with no Lanes. Lanes can then be added.
     *
     * @param name the name of the Road
     * @param map the map of which the Road is a part
     * @param aheadReservation the time ahead of arrival a vehicle can make a
     * reservation. May be overidden by a lane specific value (not implemented
     * as of 11/13/19).
     */
    public Road(String name, BasicMap map, Double aheadReservation) {
        this(name, new ArrayList<Lane>(), map, aheadReservation);
    }

    /**
     * Create a new Road with the given Lanes, ordered from left to right.
     *
     * @param name the name of the Road
     * @param lanes the Lanes from which to make the Road
     * @param map the Layout of which the Road is a part
     * @param maxAllowedAheadReservation the time ahead of arrival a vehicle can make a
     * reservation. May be overidden by a lane specific value (not implemented
     * as of 11/13/19).
     */
    public Road(String name, List<Lane> lanes, BasicMap map, Double maxAllowedAheadReservation) {
        this.name = name;
        this.lanes = new ArrayList<Lane>(lanes);
        this.map = map;
        this.index = roadCount++;
        this.maxAheadReservationTimeForRoad = maxAllowedAheadReservation;

        turnOnRedIncomingMemoization = new HashMap<IntersectionManager, Lane>();
        turnOnRedOutgoingMemoization = new HashMap<IntersectionManager, Lane>();
        incomingLanesWhichMightSupportTurningOnRed = new HashMap<IntersectionManager, HashMap<Lane, Integer>>();
        outgoingLanesWhichMightSupportTurningOnRed = new HashMap<IntersectionManager, HashMap<Lane, Integer>>();
        relativeLaneIndexInRoad = new HashMap<Lane, Integer>(lanes.size());

        // Now set up the proper relationships between them
        if (lanes.size() > 1) {
            Double speedToCompare = null;
            boolean copySpeedLimit = true;
            for (int i = 0; i < lanes.size() - 1; i++) {
                Lane lane = lanes.get(i);
                relativeLaneIndexInRoad.put(lane, i);
                if (relativeLaneIndexInRoad.containsKey(lane)) {
                    throw new IllegalArgumentException("Road already contains a lane provided, a lane cannot be added to a road twice.");
                }

                if (speedToCompare == null) {
                    speedToCompare = lane.getSpeedLimit();
                }
                copySpeedLimit = copySpeedLimit && (speedToCompare != lane.getSpeedLimit());

                lane.setRightNeighbor(lanes.get(i + 1));
                lanes.get(i + 1).setLeftNeighbor(lanes.get(i));
            }
            roadSpeedLimit = (copySpeedLimit && lanes.get(lanes.size() - 1).getSpeedLimit() == speedToCompare ? speedToCompare : null);
        } else if (lanes.size() == 1) {
            relativeLaneIndexInRoad.put(lanes.get(0), 0);
            roadSpeedLimit = lanes.get(0).getSpeedLimit();
        } else {
            roadSpeedLimit = null;
        }
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    /**
     * Get the maximum speed limit of any connected road.
     *
     * @return the maximum speed limit, in meters per second, of any connected
     * road
     */
    public double getMaximumConnectedSpeedLimit() {
        return map.getMaximumSpeedLimit();
    }

    //todo speed this up by memoizing at lane addition to road. Requires some logic regarding turn direction to IM mappings unless only one IM is allowed.
    /**
     * Figures out the leftmost departing lane which could allow a right turn on
     * red onto it. This function must be modified if it is to be used for
     * traffic which drives on the left side of the road (then it should be
     * looking for the rightmost departing lane allowing for left on red).
     *
     * @param IM
     * @return
     */
    public Lane getLeftmostOutgoingLaneWithNoConflictingIncomingActionsForRightOnRedFromItsRight(IntersectionManager IM) {
        if (!turnOnRedOutgoingMemoization.containsKey(IM)) {
            int fromRightCountSkippingClosedLanes = 0;
            HashMap<Lane, Integer> laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed = new HashMap<Lane, Integer>();

            Lane nextLane = (lanes == null || lanes.isEmpty() ? null : lanes.get(lanes.size() - 1));
            if (nextLane == null || nextLane.hasRightNeighbor()) {
                throw new RuntimeException("Rightmost lane seems to have neighbor lane to the right. This should be impossible.");
            }

            Lane leftmost = null;
            while (nextLane != null) {
                Set<Lane> lanesLeadingToLeftmost = IM.getLanesWhichHaveAnExitLaneMappingByAnyActionAndAnyVehicleTypeToLane(nextLane);

                //todo, if this is ever made generic Constants.TurnDirection.RIGHT should be changed to Constants.WITH_TRAFFIC_TURN_DIRECTION
                if (lanesLeadingToLeftmost != null && (lanesLeadingToLeftmost.size() > 1 || lanesLeadingToLeftmost.iterator().next().getLaneIM().getMappedExitLane(IM, Constants.TurnDirection.RIGHT) != nextLane)) {
                    leftmost = nextLane;
                    laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed.put(leftmost, fromRightCountSkippingClosedLanes++);
                    break; //we've found a lane with a through or left turn incoming to it, so no lane left of this lane is safe to make a right on red onto it
                } else if (lanesLeadingToLeftmost != null) { //lanesLeadingToLeftmost would be null if the lane is closed
                    //we've either found a lane that qualifies
                    leftmost = nextLane;
                    laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed.put(leftmost, fromRightCountSkippingClosedLanes++);
                } else {
                    //if no lanes lead to this lane, do nothing with it (it's a closed outgoing lane)
                }
                nextLane = nextLane.getLeftNeighbor();
            }

            outgoingLanesWhichMightSupportTurningOnRed.put(IM, laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed);
            turnOnRedOutgoingMemoization.put(IM, leftmost);
        }

        return turnOnRedOutgoingMemoization.get(IM);
    }

    /**
     * Figures out the leftmost incoming lane which could allow a right turn on
     * red onto it. This function must be modified if it is to be used for
     * traffic which drives on the left side of the road (then it should be
     * looking for the rightmost incoming lane allowing for left on red).
     *
     * @param IM
     * @return
     */
    public Lane getLeftmostIncomingLaneWithNoConflictingOutgoingActionsForRightOnRedFromItsRight(IntersectionManager IM) {
        if (!turnOnRedIncomingMemoization.containsKey(IM)) {
            int fromRightCountSkippingClosedLanes = 0;
            HashMap<Lane, Integer> laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed = new HashMap<Lane, Integer>();

            Lane nextLane = (lanes == null || lanes.isEmpty() ? null : lanes.get(lanes.size() - 1));
            if (nextLane == null || nextLane.hasRightNeighbor()) {
                throw new RuntimeException("Rightmost lane seems to have neighbor lane to the right. This should be impossible.");
            }

            Lane leftmost = null;
            while (nextLane != null) {
                Set<Constants.TurnDirection> actionsFromLane = nextLane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(IM);
                //todo, if this is ever made generic Constants.TurnDirection.LEFT should be changed to Constants.CROSS_TURN_DIRECTION and Constants.TurnDirection.RIGHT should be changed to Constants.WITH_TRAFFIC_TURN_DIRECTION
                //if the lane we're looking at is not closed (not null), and some actions are allowed but a right turn is not, revert to the previously found valid right turn lane
                if (actionsFromLane != null && !actionsFromLane.equals(Collections.EMPTY_SET) && !actionsFromLane.contains(Constants.TurnDirection.RIGHT)) {
                    break;
                } else if (actionsFromLane != null && (actionsFromLane.equals(Collections.EMPTY_SET) || actionsFromLane.contains(Constants.TurnDirection.LEFT) || actionsFromLane.contains(Constants.TurnDirection.STRAIGHT))) {
                    //we've found a lane that is open which allows a through or left turn outgoing from it and also allows a right turn, so it is the lane we're looking for
                    leftmost = nextLane;
                    laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed.put(leftmost, fromRightCountSkippingClosedLanes++);
                    break;
                } else {
                    //actionsFromLane is null meaning the lane is closed or the lane only allowed a right turn so another lane to the left might be able to turn. We need to check the next lane to the left.
                    if (actionsFromLane != null) {
                        //we didn't terminate on this lane, so it's a candidate in case we have to roll back
                        leftmost = nextLane;
                        laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed.put(leftmost, fromRightCountSkippingClosedLanes++);
                    }
                    nextLane = nextLane.getLeftNeighbor();
                }
            }

            incomingLanesWhichMightSupportTurningOnRed.put(IM, laneToIndexSkippingClosedLanesFromRightOfLanesThatMightSupportTurnOnRed);
            turnOnRedIncomingMemoization.put(IM, leftmost);
        }
        return turnOnRedIncomingMemoization.get(IM);

    }

    public int getNumberOfOutgoingLanesWhichMightSupportTurnsOnRed(IntersectionManager IM) {
        if (!turnOnRedOutgoingMemoization.containsKey(IM)) {
            getLeftmostOutgoingLaneWithNoConflictingIncomingActionsForRightOnRedFromItsRight(IM);
        }
        return (outgoingLanesWhichMightSupportTurningOnRed.containsKey(IM) ? outgoingLanesWhichMightSupportTurningOnRed.get(IM).size() : 0);
    }

    public Integer getOutgoingLaneByClosedLaneSkippingIndexFromRightWhichMightSupportTurnsOnRed(IntersectionManager IM, Lane ln) {
        if (!turnOnRedOutgoingMemoization.containsKey(IM)) {
            getLeftmostOutgoingLaneWithNoConflictingIncomingActionsForRightOnRedFromItsRight(IM);
        }
        HashMap<Lane, Integer> innerMap = outgoingLanesWhichMightSupportTurningOnRed.get(IM);
        if (innerMap == null) {
            return null;
        }

        Integer closedLaneSkippingIndex = innerMap.get(ln);

        //could be null
        return closedLaneSkippingIndex;
    }

    public int getNumberOfIncomingLanesWhichMightSupportTurnsOnRed(IntersectionManager IM) {
        if (!turnOnRedIncomingMemoization.containsKey(IM)) {
            getLeftmostIncomingLaneWithNoConflictingOutgoingActionsForRightOnRedFromItsRight(IM);
        }
        return (incomingLanesWhichMightSupportTurningOnRed.containsKey(IM) ? incomingLanesWhichMightSupportTurningOnRed.get(IM).size() : 0);
    }

    public Integer getIncomingLaneByClosedLaneSkippingIndexFromRightWhichMightSupportTurnsOnRed(IntersectionManager IM, Lane ln) {
        if (!turnOnRedIncomingMemoization.containsKey(IM)) {
            getLeftmostIncomingLaneWithNoConflictingOutgoingActionsForRightOnRedFromItsRight(IM);
        }
        HashMap<Lane, Integer> innerMap = incomingLanesWhichMightSupportTurningOnRed.get(IM);
        if (innerMap == null) {
            return null;
        }

        Integer closedLaneSkippingIndex = innerMap.get(ln);

        //could be null
        return closedLaneSkippingIndex;
    }

    public void resetMemoizationCaches() {
        turnOnRedIncomingMemoization = new HashMap<IntersectionManager, Lane>();
        turnOnRedOutgoingMemoization = new HashMap<IntersectionManager, Lane>();
        incomingLanesWhichMightSupportTurningOnRed = new HashMap<IntersectionManager, HashMap<Lane, Integer>>();
        outgoingLanesWhichMightSupportTurningOnRed = new HashMap<IntersectionManager, HashMap<Lane, Integer>>();
    }

    /**
     *
     * @return road speed limit if all lanes share the same speed limit, else
     * null
     */
    public Double getRoadSpeedLimit() {
        return roadSpeedLimit;
    }

    /**
     * Get the Lanes that make up this Road, in order from left to right.
     *
     * @return the Lanes that make up this Road, in order from left to right
     */
    public List<Lane> getLanes() {
        return Collections.unmodifiableList(lanes);
    }

    /**
     * Get the leftmost Lane in this Road.
     *
     * @return the leftmost Lane in this Road
     */
    public Lane getIndexLane() {
        if (lanes.isEmpty()) {
            return null;
        }
        return lanes.get(0);
    }

    /**
     * Get the Road that follows this Road in the opposite direction.
     *
     * @return the Road that follows this Road in the opposite direction
     */
    public Road getDual() {
        return dual;
    }

    /**
     * Set the Road that follows this Road in the opposite direction.
     *
     * @param dual the Road that follows this Road in the opposite direction
     */
    public void setDual(Road dual) {
        this.dual = dual;
        // Also make sure the reciprocal relationship is set.
        dual.dual = this;
    }

    /**
     * Whether or not this Road has a dual.
     *
     * @return whether or not this Road has a dual
     */
    public boolean hasDual() {
        return dual != null;
    }

    /**
     * Add a right most lane to this Road.
     *
     * @param lane the Lane to add
     */
    public void addTheRightMostLane(Lane lane) {
        if (!lanes.isEmpty()) {
            if (relativeLaneIndexInRoad.containsKey(lane)) {
                throw new IllegalArgumentException("Road already contains a lane provided, a lane cannot be added to a road twice.");
            }

            relativeLaneIndexInRoad.put(lane, lanes.size());
            Lane rightmost = lanes.get(lanes.size() - 1);
            rightmost.setRightNeighbor(lane);
            lane.setLeftNeighbor(rightmost);
            if (roadSpeedLimit != lane.getSpeedLimit()) {
                roadSpeedLimit = null; //clear this as the road itself doesn't really have a speed limit, the lanes do.
            }
        } else {
            relativeLaneIndexInRoad.put(lane, 0);
            roadSpeedLimit = lane.getSpeedLimit();
        }
        resetMemoizationCaches();
        lanes.add(lane);
    }

    /**
     * Get the relative index of a lane in this road. 0 is the leftmost, 1 right
     * of that, etc.
     *
     * @param lane
     * @return
     */
    public int getRelativeIndexOfLaneInRoad(Lane lane) {
        if (!relativeLaneIndexInRoad.containsKey(lane)) {
            return -1;
        } else {
            return relativeLaneIndexInRoad.get(lane);
        }
    }

    /**
     * Get the name of this Road. An alias for {@link #getName()}.
     *
     * @return the name of this Road
     */
    @Override
    public String toString() {
        return getName();
    }

    /**
     * Get the name of this Road.
     *
     * @return the name of this Road
     */
    public String getName() {
        return name;
    }

    /**
     * Get number of lanes on this road.
     *
     * @return the number of lane objects in this road defined as the number of
     * elements in the lanes list.
     */
    public int getNumberOfLanes() {
        return lanes.size();
    }

    /**
     *
     * @return the maximum possible time ahead of arrival a vehicle can make a
     * reservation on this road. This may be restricted even further by
     * intersection. This theoretically may be overidden by a lane specific
     * value (not implemented as of 11/13/19).
     */
    public Double getAheadReservationTimeAllowedForRoadWithoutIntersectionSpecificAdjustment() {
        return maxAheadReservationTimeForRoad;
    }
}
