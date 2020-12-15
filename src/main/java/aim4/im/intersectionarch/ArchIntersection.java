package aim4.im.intersectionarch;

import aim4.config.Constants;
import aim4.config.SimConfig;
import aim4.im.Intersection;
import aim4.map.Road;
import aim4.map.lane.Lane;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import aim4.util.Util;

public class ArchIntersection {

    /**
     * Used to map the speed limits for given roads for the given intersection.
     */
    private Map<Constants.Direction, Double> speedLimits;
    /**
     * Used to force roads to be long enough so that a vehicle on the slowest
     * road couldn't create a reservation such that a vehicle spawned on a
     * faster road could be spawned afterward and get to the intersection at the
     * same time. Note, this is used to effectively ensure the no lane
     * change/vehicle detection area is adequately sized in the case that comm
     * distance is infinite (i.e., vehicles will make a reservation as soon as
     * they spawn)
     */
    private double minSpeedLimit;

    /**
     * Used to force roads to be long enough so that a vehicle on the slowest
     * road couldn't create a reservation such that a vehicle spawned on a
     * faster road could be spawned afterward and get to the intersection at the
     * same time. Note, this is used to effectively ensure the no lane
     * change/vehicle detection area is adequately sized in the case that comm
     * distance is infinite (i.e., vehicles will make a reservation as soon as
     * they spawn)
     */
    private double maxSpeedLimit;
    /**
     * Used to map the ahead reservation times for given roads for the given
     * intersection.
     */
    private Map<Constants.Direction, Double> aheadReservationTimes;
    /**
     * Used to map an entering lane to an exit lane by turn directions.
     */
    private Map<Lane, Map<Constants.TurnDirection, Lane>> exitLanes;
    /**
     * Used to map an entering lane's turning policy for particular vehicle
     * types.
     */
    private Map<Lane, Map<SimConfig.VEHICLE_TYPE, Set<Constants.TurnDirection>>> turnPolicies;
    /**
     * Logs which lanes have default mappings for turn policies and exit lanes
     */
    private Set<Lane> lanesDefaultMapping;
    /**
     * Logs if a complete setup of this ArchIntersection has been performed, or
     * if only speed limits have been stored.
     */
    private boolean isSetup;
    /**
     * Caches road specs for if the intersection wasn't provided, and setup
     * needs to be performed fully after construction.
     */
    private final EnumMap<Constants.Direction, ArchRoadSpec> roadSpecs;
    /**
     * Caches the maximum number of lanes (incoming or outgoing) compared on all
     * roads specified in roadSpecs. -1 means no lanes were specified.
     */
    private final int maxLanesAtIntersection;

    /**
     * Constructs an ArchIntersection with the provided information. You must
     * call tieIntersection to be able to retrieve any information other than
     * speed limits from this ArchIntersection when this constructor is used.
     *
     * @param roadSpecs Map of road cardinal direction to spec for road (in
     * lanes, out lanes, and speed limit)
     */
    public ArchIntersection(EnumMap<Constants.Direction, ArchRoadSpec> roadSpecs) {
        speedLimits = new EnumMap<Constants.Direction, Double>(Constants.Direction.class);
        aheadReservationTimes = new EnumMap<Constants.Direction, Double>(Constants.Direction.class);
        int maxLanes = -1;

        minSpeedLimit = -1;
        maxSpeedLimit = -1;
        if (roadSpecs != null) {
            this.roadSpecs = roadSpecs;
            for (ArchRoadSpec roadSpec : roadSpecs.values()) {

                if (roadSpec != null) {
                    //set speed limit for road
                    Constants.Direction dir = roadSpec.getDirection();
                    speedLimits.put(dir, roadSpecs.get(dir).getSpeed());
                    //this will null pointer exception if a speed limit value is missing
                    minSpeedLimit = Math.min(minSpeedLimit, roadSpecs.get(dir).getSpeed());
                    maxSpeedLimit = Math.max(maxSpeedLimit, roadSpecs.get(dir).getSpeed());
                    aheadReservationTimes.put(dir, roadSpecs.get(dir).getAheadReservationTime());
                    maxLanes = Math.max(maxLanes, Math.max(roadSpec.getInLanesCount(), roadSpec.getOutLanesCount()));
                }
            }
        } else {
            throw new RuntimeException("ArchRoadSpecs map passed into ArchIntersection was null. This is not allowed.");
        }
        maxLanesAtIntersection = maxLanes;
        isSetup = false;
    }

    /**
     * Constructs an ArchIntersection for a given intersection with the provided
     * information.
     *
     * @param inter Intersection object for the intersection this
     * ArchitectureDTO is to be configured for.
     * @param roadSpecs Map of road cardinal direction to spec for road (in
     * lanes, out lanes, and speed limit)
     */
    public ArchIntersection(Intersection inter, EnumMap<Constants.Direction, ArchRoadSpec> roadSpecs) {
        this(roadSpecs);
        setup(inter);
    }

    /**
     * Handles verifying that appropriate maps for exitLanes and turnPolicies
     * exist and are populated.
     *
     * @param inLane The incoming lane to the intersection
     * @param outLane The desired outgoing lane to be mapped to the incoming
     * lane
     * @param vType The vehicle type for which the mapping should apply
     * @param turnDir the direction to turn (Left, right, etc.) from the
     * incoming lane to the outgoing one
     */
    private void addExitLanesAndTurnPolicies(Lane inLane, Lane outLane, SimConfig.VEHICLE_TYPE vType, Constants.TurnDirection turnDir) {
        Map<Constants.TurnDirection, Lane> turnToExit;
        Map<SimConfig.VEHICLE_TYPE, Set<Constants.TurnDirection>> typeToDirection;

        if (!exitLanes.containsKey(inLane)) {
            exitLanes.put(inLane, new EnumMap<Constants.TurnDirection, Lane>(Constants.TurnDirection.class));
        }
        turnToExit = exitLanes.get(inLane);

        if (!turnPolicies.containsKey(inLane)) {
            turnPolicies.put(inLane, new EnumMap<SimConfig.VEHICLE_TYPE, Set<Constants.TurnDirection>>(SimConfig.VEHICLE_TYPE.class));
        }
        typeToDirection = turnPolicies.get(inLane);

        if (!turnToExit.containsKey(turnDir)) {
            turnToExit.put(turnDir, outLane);
        } else if (turnToExit.get(turnDir) != outLane) {
            throw new RuntimeException("Duplicate and conflicting mapping of incoming lane to outgoing lane on incoming road " + inLane.getContainingRoad().getName() + " to outgoing road " + outLane.getContainingRoad().getName()
                    + ". Existing mapping: " + inLane.getIndexInRoad() + "=>" + turnDir.name() + "=>" + turnToExit.get(turnDir).getIndexInRoad() + ") Conflicting mapping: " + inLane.getIndexInRoad() + "=>" + turnDir.name() + "=>" + outLane.getIndexInRoad() + ")");
        }

        if (!typeToDirection.containsKey(vType)) {
            EnumSet<Constants.TurnDirection> allowedTurns = EnumSet.noneOf(Constants.TurnDirection.class);
            allowedTurns.add(turnDir);
            typeToDirection.put(vType, allowedTurns);
        } else {
            typeToDirection.get(vType).add(turnDir);
        }

        Constants.TurnDirection[] composedTurns = Util.getComposedActions(turnDir);
        if (composedTurns != null && composedTurns.length > 0 && !(composedTurns.length == 1 && composedTurns[0] == turnDir)) {
            for (Constants.TurnDirection compTurn : composedTurns) {
                //todo make this more effecient?
                boolean containsAll = true;
                Set<Constants.TurnDirection> tdsForType = typeToDirection.get(vType);
                if (!tdsForType.contains(compTurn)) {
                    for (Constants.TurnDirection td : Util.getDecomposedActions(compTurn)) {
                        if (!tdsForType.contains(td)) {
                            containsAll = false;
                            break;
                        }
                    }
                }
                if (containsAll) {
                    typeToDirection.get(vType).add(compTurn);
                }
            }
        }
    }

    /**
     * Goes through the lane lanes in a road to return an ArrayList (size
     * provided) of the lanes in order of the leftmost being at index 0 and
     * index increasing as you move right a lane. This function is to simplify
     * calculations in the constructor over potentially more inline methods
     * using lane.getIndexInRoad();
     *
     * @param rd Road from which to fetch the lanes.
     * @param startInd the relative index you wish to start at in the road with
     * the leftmost lane being at index 0.
     * @param numberOfLane the number of lanes you wish to retrieve.
     * @return An ArrayList of lanes in relative order with 0 being the leftmost
     * (starting at startInd).
     */
    private ArrayList<Lane> getLanesWithRelIndex(Road rd, int startInd, int numberOfLanes) {
        ArrayList<Lane> lanesByIndex = new ArrayList<Lane>(numberOfLanes);
        Lane ln = rd.getIndexLane();
        int nextIndex = 1;
        while (nextIndex < startInd + 1 && ln != null) {
            ln = ln.getRightNeighbor();
            ++nextIndex;
        }

        while (nextIndex < numberOfLanes + startInd && ln != null) {
            lanesByIndex.add(ln);
            ln = ln.getRightNeighbor();
            ++nextIndex;
        }
        if (ln != null) {
            lanesByIndex.add(ln);
        }

        if (lanesByIndex.size() != numberOfLanes) {
            throw new RuntimeException("Unable to fetch lanes with relative indices because of invalid parameters: start index: " + startInd + ", number of lanes to return: " + numberOfLanes + ", number of lanes: " + lanesByIndex.size() + ", max number of lanes: " + rd.getLanes().size());
        }

        return lanesByIndex;
    }

    /**
     * Checks provided map of ArrayList of Lanes by relative index to see if
     * element following the parameters exists. If not, creates one and then
     * returns it. If so, returns it.
     *
     * @param lanesByRelIndex map of ArrayList of Lanes by relative index
     * @param dir Direction the road for this lane follows
     * @param rd The road object itself
     * @param straightLaneOffset offset for relative indices for lanes to go
     * straight (as a car can't change lanes when going straight through an
     * intersection)
     * @param numOfLanes the number of lanes that should be retrieved
     * @param isSmallSide used to execute logic based on the comparison of the
     * road's number of lanes on both sides of the intersection
     * @return ArrayList of Lanes in relative order in the road from left to
     * right with left being at index 0.
     */
    private ArrayList<Lane> getCachedLanesWithRelIndex(EnumMap<Constants.Direction, ArrayList<Lane>> lanesByRelIndex, Constants.Direction dir, Road rd, int straightLaneOffset, int numOfLanes, boolean isSmallSide) {
        ArrayList<Lane> relIndices;
        if (lanesByRelIndex.containsKey(dir)) {
            relIndices = lanesByRelIndex.get(dir);
        } else {
            if (isSmallSide) {
                relIndices = getLanesWithRelIndex(rd, Math.abs(straightLaneOffset), numOfLanes);
            } else {
                relIndices = getLanesWithRelIndex(rd, 0, numOfLanes);
            }
            lanesByRelIndex.put(dir, relIndices);
        }
        return relIndices;
    }

    /**
     * Gets the speed limit for the specified road at the intersection for which
     * this ArchIntersection is configured. CAUTION: This checks purely based on
     * direction.
     *
     * @param dir Direction of the Road for which the speed limit needs to be
     * checked.
     * @return <code>null</code> if direction is not mapped for this
     * ArchIntersection's intersection or no speed limit was mapped for
     * direction. Otherwise returns the speed limit. CAUTION: This checks purely
     * based on direction.
     */
    public Double getSpeedLimit(Constants.Direction dir) {
        return speedLimits.get(dir);
    }

    /**
     * Gets the speed limit for the specified road at the intersection for which
     * this ArchIntersection is configured. CAUTION: This checks purely based on
     * direction.
     *
     * @param heading Heading in radians for which any mapped speed limits
     * (based on the nearest Constants.Direction) need to be fetched.
     * @return <code>null</code> if direction is not mapped for this
     * ArchIntersection's intersection or no speed limit was mapped for
     * direction. Otherwise returns the speed limit. CAUTION: This checks purely
     * based on direction.
     */
    public Double getSpeedLimit(Double heading) {
        return speedLimits.get(Util.getDirectionFromHeadingCardinalOrIntercardinal(heading));
    }

    /**
     * Gets the ahead reservation time for the specified road at the
     * intersection for which this ArchIntersection is configured. CAUTION: This
     * checks purely based on direction.
     *
     * @param dir Direction of the Road for which the ahead reservation time
     * needs to be checked.
     * @return <code>null</code> if direction is not mapped for this
     * ArchIntersection's intersection. Otherwise returns the ahead reservation
     * time. CAUTION: This checks purely based on direction.
     */
    public Double getAheadReservationTime(Constants.Direction dir) {
        return aheadReservationTimes.get(dir);
    }

    /**
     * Gets the ahead reservation time for the specified road at the
     * intersection for which this ArchIntersection is configured. CAUTION: This
     * checks purely based on direction.
     *
     * @param heading Heading in radians for which any mapped ahead reservation
     * time (based on the nearest Constants.Direction) need to be fetched.
     * @return <code>null</code> if direction is not mapped for this
     * ArchIntersection's intersection. Otherwise returns the ahead reservation
     * time. CAUTION: This checks purely based on direction.
     */
    public Double getAheadReservationTime(Double heading) {
        return aheadReservationTimes.get(Util.getDirectionFromHeadingCardinalOrIntercardinal(heading));
    }

    /**
     * Gets the exiting lane for the provided parameters if mapped. Use
     * isDefaultMapping to check if mapping indicates default mapping or none
     * mapping.
     *
     * @param lane Lane entering the intersection.
     * @param td Direction a vehicle will be turning/proceeding
     * @return Returns the exiting lane for the parameters provided,
     * <code>null</code> if entering the intersection with the provided turn
     * action is disabled or if no lane has been mapped.
     */
    public Lane getExitLane(Lane lane, Constants.TurnDirection td) {
        if (!isSetup) {
            throw new UnsupportedOperationException("Getting/retrieving members prior to complete setup is not supported. Please call tieIntersection before getting/retrieving parameters since the delayed setup form of the constructor was used.");
        }
        if (exitLanes.containsKey(lane)) {
            return exitLanes.get(lane).get(td);
        }
        return null;
    }

    /**
     * Returns determination of if mapping indicates default mapping or none
     * mapping/nonexistence.
     *
     * @param lane Lane entering the intersection.
     * @return Returns <code>true</code> if default mapping was applied for this
     * lane (both exit lane and turn policies). Returns <code>false</code>
     * otherwise.
     */
    public boolean isDefaultMapping(Lane lane) {
        if (!isSetup) {
            throw new UnsupportedOperationException("Getting/retrieving members prior to complete setup is not supported. Please call tieIntersection before getting/retrieving parameters since the delayed setup form of the constructor was used.");
        }
        return lanesDefaultMapping.contains(lane);
    }

    /**
     * Returns all the turn directions that have been mapped for a given lane
     * for exit mapping.
     *
     * @param lane The incoming lane to the intersection.
     * @return All the turn directions that have been mapped for the incoming
     * lane.
     */
    public Iterable<Constants.TurnDirection> getExitMappingKeys(Lane lane) {
        if (!isSetup) {
            throw new UnsupportedOperationException("Getting/retrieving members prior to complete setup is not supported. Please call tieIntersection before getting/retrieving parameters since the delayed setup form of the constructor was used.");
        }
        if (exitLanes.containsKey(lane)) {
            return exitLanes.get(lane).keySet();
        }
        return null;
    }

    /**
     * Returns all the vehicle types that have been mapped for a given lane for
     * turn policies.
     *
     * @param lane The incoming lane to the intersection.
     * @return All the vehicle types that have been mapped for the incoming
     * lane.
     */
    public Iterable<SimConfig.VEHICLE_TYPE> getTurnPolicyKeys(Lane lane) {
        if (!isSetup) {
            throw new UnsupportedOperationException("Getting/retrieving members prior to complete setup is not supported. Please call tieIntersection before getting/retrieving parameters since the delayed setup form of the constructor was used.");
        }
        if (turnPolicies.containsKey(lane)) {
            return turnPolicies.get(lane).keySet();
        }
        return null;
    }

    /**
     * Gets the allowed turns for the provided parameters if mapped. Use
     * isDefaultMapping to check if mapping indicates default mapping or none
     * mapping.
     *
     * @param lane Lane entering the intersection.
     * @param vType Type of vehicle
     * @return Returns a set of the allowed turns for the parameters provided,
     * <code>null</code> if entering the intersection with the provided vehicle
     * type is disabled or if no turns have been mapped.
     */
    public Set<Constants.TurnDirection> getAllowedTurns(Lane lane, SimConfig.VEHICLE_TYPE vType) {
        if (!isSetup) {
            throw new UnsupportedOperationException("Getting/retrieving members prior to complete setup is not supported. Please call tieIntersection before getting/retrieving parameters since the delayed setup form of the constructor was used.");
        }
        if (turnPolicies.containsKey(lane)) {
            return turnPolicies.get(lane).get(vType);
        }
        return null;
    }

    /**
     * Gets the maximum number of lanes (max(incoming, outgoing)) for a road.
     *
     * @param roadDir Direction of the road to check for.
     * @return Max of the incoming and outgoing number of lanes for the road
     * provided. -1 means none were specified for the road or the road wasn't
     * specified in this ArchIntersection.
     */
    public int getMaxNumberOfLanesInRoad(Constants.Direction roadDir) {
        ArchRoadSpec rs = roadSpecs.get(roadDir);
        if (rs == null) {
            return -1;
        } else {
            return Math.max(rs.getInLanesCount(), rs.getOutLanesCount());
        }
    }

    /**
     * Gets the maximum number of lanes (max(incoming, outgoing)) across all
     * roads handled by this ArchIntersection.
     *
     * @return Max of the incoming and outgoing number of lanes across all roads
     * at this intersection specified in the constraints. -1 means none were
     * specified.
     */
    public int getMaxNumberOfLanes() {
        return maxLanesAtIntersection;
    }

    /**
     * Public wrapper around private setup which sets up all mappings for this
     * ArchIntersection to prepare for use in setting lane and road parameters.
     *
     * @param inter Intersection to be used when mapping parameters for this
     * ArchIntersection
     */
    public void tieIntersection(Intersection inter) {
        setup(inter);
    }

    /**
     * Sets up all mappings for this ArchIntersection to prepare for use in
     * setting lane and road parameters.
     *
     * @param inter Intersection to be used when mapping parameters for this
     * ArchIntersection
     */
    private void setup(Intersection inter) {
        if (!isSetup) {
            speedLimits = new EnumMap<Constants.Direction, Double>(Constants.Direction.class);
            aheadReservationTimes = new EnumMap<Constants.Direction, Double>(Constants.Direction.class);
            exitLanes = new HashMap<Lane, Map<Constants.TurnDirection, Lane>>();
            turnPolicies = new HashMap<Lane, Map<SimConfig.VEHICLE_TYPE, Set<Constants.TurnDirection>>>();
            lanesDefaultMapping = new HashSet<Lane>();

            HashMap<Road, Constants.Direction> roadToDir = new HashMap<Road, Constants.Direction>();
            HashMap<Constants.Direction, Road> dirToRoad = new HashMap<Constants.Direction, Road>();
            //cache roads by direction
            for (Road rd : inter.getRoads()) {
                Constants.Direction roadDir = Util.getDirectionFromHeadingCardinalOrIntercardinal(inter.getEntryHeading(rd.getIndexLane()));
                if (!roadToDir.containsKey(rd)) {
                    roadToDir.put(rd, roadDir);
                    dirToRoad.put(roadDir, rd);
                } else {
                    throw new RuntimeException("Duplicate road or road direction key found when attempting to populate ArchitectureDTO: " + rd.getName() + " " + roadDir + ". Duplicate directions are not currently supported.");
                }
            }

            //cache for road lanes/indices
            EnumMap<Constants.Direction, ArrayList<Lane>> inLanesByRelIndex = new EnumMap<Constants.Direction, ArrayList<Lane>>(Constants.Direction.class);
            EnumMap<Constants.Direction, ArrayList<Lane>> outLanesByRelIndex = new EnumMap<Constants.Direction, ArrayList<Lane>>(Constants.Direction.class);
            //actually set up the spec
            //go through each entering road in the intersection. It doesn't make sense to look at outgoing roads, because any mapping for interesection architecture should be handled by the incoming roads. (Even though a road may be both outgoing and incoming.)
            for (Road inRoad : inter.getEntryRoads()) {
                //get direction of the road we're looking at
                Constants.Direction inDir = roadToDir.get(inRoad);
                //get spec for the road
                ArchRoadSpec rdSpec = roadSpecs.get(inDir);

                if (rdSpec != null) {
                    //set speed limit for road
                    speedLimits.put(inDir, roadSpecs.get(inDir).getSpeed());

                    //set ahead reservation time for road
                    aheadReservationTimes.put(inDir, roadSpecs.get(inDir).getAheadReservationTime());

                    //caching which lanes were actually specified in file, setting default values for others
                    HashSet<Lane> checkedLanes = new HashSet<Lane>();
                    //get lanes in index order. Also, naively shoves the incoming lanes onto to the leftmost number of lanes suiting the numbers provided. (note: straight lanes *must* remain at their true relative index, they cannot shift in the middle of an intersection)
                    ArrayList<Lane> relInLanesIndices = getCachedLanesWithRelIndex(inLanesByRelIndex, inDir, inRoad, rdSpec.getStraightLaneOffset(), rdSpec.getInLanesCount(), rdSpec.getInLanesCount() < rdSpec.getOutLanesCount());

                    //look at the spec for each lane
                    for (ArchLaneSpec laneSpec : rdSpec.getlaneSpecs()) {

                        //for each outgoing direction, and vehicle type combination, look at the turn mappings and assign them
                        Map<Constants.Direction, EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>>> turnPolicyAndMapping = laneSpec.getTurnPoliciesAndRestrictions();
                        for (Constants.Direction outDir : turnPolicyAndMapping.keySet()) {
                            EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>> vTypeToTurns = turnPolicyAndMapping.get(outDir);
                            Road outRoad = dirToRoad.get(outDir);
                            for (SimConfig.VEHICLE_TYPE vType : vTypeToTurns.keySet()) {
                                Map<Integer, Integer> inToOut = vTypeToTurns.get(vType);
                                for (Integer inLaneIndex : inToOut.keySet()) {
                                    Lane inLane;
                                    Lane outLane;
                                    int outLaneIndex = inToOut.get(inLaneIndex);
                                    ArchRoadSpec outSpec = roadSpecs.get(outDir);
                                    ArrayList<Lane> relOutLanesIndices;
                                    if (outDir != null) {
                                        relOutLanesIndices = getCachedLanesWithRelIndex(outLanesByRelIndex, outDir, outRoad, outSpec.getStraightLaneOffset(), outSpec.getOutLanesCount(), outSpec.getOutLanesCount() < outSpec.getInLanesCount());
                                    } else {
                                        relOutLanesIndices = getCachedLanesWithRelIndex(outLanesByRelIndex, outDir, outRoad, 0, outRoad.getNumberOfLanes(), false);
                                    }
                                    if (relInLanesIndices.size() > inLaneIndex && relOutLanesIndices.size() > outLaneIndex) {
                                        inLane = relInLanesIndices.get(inLaneIndex);
                                        outLane = relOutLanesIndices.get(outLaneIndex);
                                    } else {
                                        throw new RuntimeException("Invalid number of lanes specified. Number of lanes for incoming road " + inRoad.getName() + " is " + inRoad.getNumberOfLanes() + ", requested is " + (inLaneIndex + 1)
                                                + ". Number of lanes for outgoing road" + outRoad.getName() + " is " + outRoad.getNumberOfLanes() + ", requested is " + (outLaneIndex + 1));
                                    }
                                    checkedLanes.add(inLane);
                                    inLane.getLaneIM().setWithTrafficTurnOnRedAllowed(rdSpec.getSetOfLanesPotentiallySupportingWithTrafficTurnsOnRed().contains(inLaneIndex)); //this is redundant, as it gets done on every loop that comes across this lane, but we'll leave it for now.
                                    addExitLanesAndTurnPolicies(inLane, outLane, vType, inter.calcTurnDirection(inLane, outLane));
                                }
                            }
                        }
                    }

                    for (Lane ln : inRoad.getLanes()) {
                        if (!checkedLanes.contains(ln)) {
                            lanesDefaultMapping.add(ln);
                        }
                    }
                }
            }
            isSetup = true;
        }
    }

    /**
     * Gets if this ArchIntersection has been fully setup.
     *
     * @return true if this ArchIntersection has been fully set up, false
     * otherwise.
     */
    public boolean isSetup() {
        return isSetup;
    }

    public double getMinSpeedLimit() {
        return minSpeedLimit;
    }

    public double getMaxSpeedLimit() {
        return maxSpeedLimit;
    }
}
