package aim4.map.trafficbyturns;

import aim4.config.Constants;
import aim4.config.Debug;
import aim4.config.Resources;
import aim4.config.SimConfig;
import aim4.im.IntersectionManager;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.destination.FileBasedDestinationSelector;
import aim4.map.lane.Lane;
import aim4.map.lane.LaneIM;
import aim4.sim.AutoDriverOnlySimulator;
import aim4.util.Util;
import aim4.vehicle.VehicleSpecDatabase;
import expr.trb.DesignatedLanesExpr;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class LaneRestrictedFileSpawnSpecGenerator extends DestinationFileSpawnSpecGenerator {

    /**
     * Used to keep track of the spawn points that have spawned in a single time
     * step.
     */
    protected HashSet<SpawnPoint> spawnedThisStep;
    protected HashMap<IntersectionManager, HashMap<Road, EnumMap<SimConfig.VEHICLE_TYPE, EnumMap<Constants.TurnDirection, HashSet<Lane>>>>> potentiallyUsableLanesForTurningActionAndType;

    public LaneRestrictedFileSpawnSpecGenerator(Map<Constants.Direction, ArrayList<TurnMovementCountForRoad>> turncounts, FileBasedDestinationSelector fbds) {
        super(turncounts, fbds);
        if (Debug.CAN_CHANGE_LANE) {
            throw new RuntimeException("Error: LaneRestrictedFileSpawnSpecGenerator cannot be used while lane changes are allowed.");
        }
        spawnedThisStep = new HashSet<SpawnPoint>();
    }

    /////////////////////////////////
    // PUBLIC FIELDS
    /////////////////////////////////
    //current implementation assumes each spawn point will only be sent/acted upon once per time step
    @Override
    public List<SpawnPoint.SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
        return act(spawnPoint, timeStep, SimConfig.VEHICLE_TYPE.AUTO);
    }

    //THIS ONLY FUNCTIONS WITH 1 INTERSECTION BECAUSE OF ** BELOW, also requires that the lane goes all the way to the intersection from spawn todo!!
    @Override
    public List<SpawnPoint.SpawnSpec> act(SpawnPoint spawnPoint, double timeStep, SimConfig.VEHICLE_TYPE vehicleType) {
        updateTimeTracking(spawnPoint, timeStep);
        ArrayList<SpawnPoint.SpawnSpec> specs = new ArrayList<SpawnPoint.SpawnSpec>();
        //** THIS ONLY FUNCTIONS WITH 1 INTERSECTION
        Lane spawnLane = spawnPoint.getLane();
        Road spawnroad = spawnLane.getContainingRoad();
        Constants.Direction roadDir = Util.getDirectionFromHeadingCardinalOrIntercardinal(spawnLane.getLaneIM().firstIntersectionManager().getIntersection().getEntryHeading(spawnLane));
        //since time tracking appears below to accomodate tracking of which spawn points have been selected to spawn so far, we need to use trtime+timeStep here
        SpawnScheduleInformation ssi = shouldSpawnVehicle(spawnPoint, roadDir, trtime, timeStep, vehicleType);

        if (ssi != null) {
            updateAppropriateSpawnQueue(roadDir, ssi);
            //todo!! need to add a way to specify how these are selected, rather than just random shuffling and assuming the first one will be picked
            for (int i = 0; i < VehicleSpecDatabase.getNumOfSpec(); i++) {
                specs.add(new SpawnPoint.SpawnSpec(spawnPoint.getCurrentTime(),
                        VehicleSpecDatabase.getVehicleSpecById(i),
                        destselect.selectDestination(spawnroad, ssi.getTD()), vehicleType));
            }
            Collections.shuffle(specs, Util.RANDOM_NUM_GEN);
        }
        if (!specs.isEmpty()) {
            totalSpawned.put(roadDir, totalSpawned.get(roadDir) + 1);
        }
        return specs;
    }

    //does not alow you to go backwards. Assumes once you've progressed to an index, you won't go to an earlier time.
    protected SpawnScheduleInformation shouldSpawnVehicle(SpawnPoint spawnPoint, Constants.Direction roadDir, double time, double timeStep, SimConfig.VEHICLE_TYPE vType) {
        int indx = getIndexWithTime(time);
        if (indx > lastindex) {
            updateToPresent(indx);
        }

        return getAction(spawnPoint, roadDir, time, timeStep, vType);
    }

    protected SpawnScheduleInformation getAction(SpawnPoint spawnPoint, Constants.Direction roadDir, double time, double timeStep, SimConfig.VEHICLE_TYPE vType) {
        //shortcircuiting to check if appropriate time has passed
        /*if there are spawns left for the road, and the appropriate time has passed for a spawn, signal to spawn something.*/
        if (spawnQueues.get(roadDir).size() > 0
                && time >= spawnQueues.get(roadDir).get(0).getSpawnTime()) {
            LaneIM spawnLaneIM = spawnPoint.getLane().getLaneIM();
            IntersectionManager firstIM = spawnPoint.getLane().getLaneIM().firstIntersectionManager();

            //checking for empty list of keys is implicitly handled by outermost if
            SpawnScheduleInformation schedInfo = null;
            Lane lane = spawnPoint.getLane();
            Iterator<SpawnScheduleInformation> iter = spawnQueues.get(roadDir).iterator();
            SpawnScheduleInformation candidateSpawnAppointment;
            while (iter.hasNext()) {
                candidateSpawnAppointment = iter.next();

                if (candidateSpawnAppointment.getSpawnTime() <= time) {
                    //Set<Lane> bestLanes = getBestLanes(candidateSpawnAppointment.getTD(), firstIM, lane.getContainingRoad(), vType);
                    Set<Lane> bestLanes = getAllowedLanesToSpawnOn(firstIM, candidateSpawnAppointment.getTD(), lane.getContainingRoad(), vType);
                    //greedy approach to spawning, if this lane is the best lane for any action, spawn with that action
                    if (bestLanes.contains(lane)) {
                        if (candidateSpawnAppointment.getSpawnTime() < time - timeStep) {
                            ++DesignatedLanesExpr.lateSpawns;
                            DesignatedLanesExpr.latenessString += roadDir.name() + "," + candidateSpawnAppointment.getSpawnTime() + "," + time + "," + (time - candidateSpawnAppointment.getSpawnTime()) + "\n";
                        }

                        schedInfo = candidateSpawnAppointment;
                        spawnedThisStep.add(lane.getSpawnPoint());
                        break;
                    }
                } else {
                    break;
                }
            }
            return schedInfo;

        }
        //no spawns left or appropriate time has not passed
        return null;
    }

    //todo!! AP need to cache results, maybe each lane gets assigned "best" actions for this time step, to make this more effecient
    protected Set<Lane> getBestLanes(Constants.TurnDirection td, IntersectionManager IM, Road rd, SimConfig.VEHICLE_TYPE vType) {
        int currentCount = Integer.MAX_VALUE;
        HashSet<Lane> bestLanes = new HashSet<Lane>();
        for (Lane lane : rd.getLanes()) {
            LaneIM laneIM = lane.getLaneIM();
            SpawnPoint sp = lane.getSpawnPoint();
            //check that the SpawnPoint hasn't already spawned this time step and for IM managing each lane is handled in isValidActionFromLane, which will return false if the provided IM doesn't manage the lane due to the mapping contained in the LaneIM class.
            if (!spawnedThisStep.contains(lane.getSpawnPoint()) && laneIM.isValidActionFromLane(IM, td, vType)) {
                int vecLaneCount = laneIM.getVehiclesToNextIntersection(lane.distanceAlongLane(sp.getPosition()));

                if (lane.distanceAlongLane(sp.getPosition()) < 0) {
                    throw new RuntimeException("Negative distance on lane calculated when searching for best lanes in LaneRestrictedFileSpawnSpecGenerator.");
                }

                if (vecLaneCount < currentCount) {
                    currentCount = vecLaneCount;
                    bestLanes = new HashSet<Lane>();
                    bestLanes.add(lane);
                } else if (vecLaneCount == currentCount) {
                    bestLanes.add(lane);
                }
            }
        }

        return bestLanes;
    }

    protected Set<Lane> getAllowedLanesToSpawnOn(IntersectionManager im, Constants.TurnDirection td, Road rd, SimConfig.VEHICLE_TYPE vType) {
        if (potentiallyUsableLanesForTurningActionAndType == null) {
            potentiallyUsableLanesForTurningActionAndType = new HashMap<IntersectionManager, HashMap<Road, EnumMap<SimConfig.VEHICLE_TYPE, EnumMap<Constants.TurnDirection, HashSet<Lane>>>>>();
        }
        if (!potentiallyUsableLanesForTurningActionAndType.containsKey(im)) {
            potentiallyUsableLanesForTurningActionAndType.put(im, new HashMap<Road, EnumMap<SimConfig.VEHICLE_TYPE, EnumMap<Constants.TurnDirection, HashSet<Lane>>>>());
            for (Road road : im.getIntersection().getEntryRoads()) {
                if (!potentiallyUsableLanesForTurningActionAndType.get(im).containsKey(road)) {
                    potentiallyUsableLanesForTurningActionAndType.get(im).put(road, new EnumMap<SimConfig.VEHICLE_TYPE, EnumMap<Constants.TurnDirection, HashSet<Lane>>>(SimConfig.VEHICLE_TYPE.class));
                }
                for (Lane lane : road.getLanes()) {
                    for (SimConfig.VEHICLE_TYPE profileVehicleType : SimConfig.VEHICLE_TYPE.values()) {
                        if (lane.getLaneIM().validActionsFromLane(im, profileVehicleType) != null) {
                            for (Constants.TurnDirection turnDir : lane.getLaneIM().validActionsFromLane(im, profileVehicleType)) {
                                if (!potentiallyUsableLanesForTurningActionAndType.get(im).get(road).containsKey(profileVehicleType)) {
                                    potentiallyUsableLanesForTurningActionAndType.get(im).get(road).put(profileVehicleType, new EnumMap<Constants.TurnDirection, HashSet<Lane>>(Constants.TurnDirection.class));
                                }

                                if (!potentiallyUsableLanesForTurningActionAndType.get(im).get(road).get(profileVehicleType).containsKey(turnDir)) {
                                    potentiallyUsableLanesForTurningActionAndType.get(im).get(road).get(profileVehicleType).put(turnDir, new HashSet<Lane>());
                                }

                                potentiallyUsableLanesForTurningActionAndType.get(im).get(road).get(profileVehicleType).get(turnDir).add(lane);
                            }
                        }
                    }
                }
            }
        }

        if (potentiallyUsableLanesForTurningActionAndType.get(im).containsKey(rd) && potentiallyUsableLanesForTurningActionAndType.get(im).get(rd).containsKey(vType) && potentiallyUsableLanesForTurningActionAndType.get(im).get(rd).get(vType).containsKey(td)) {
            HashSet<Lane> potentialLanes = potentiallyUsableLanesForTurningActionAndType.get(im).get(rd).get(vType).get(td);

            return Collections.unmodifiableSet(potentialLanes);
        } else {
            return Collections.EMPTY_SET;
        }

    }

    /**
     * Check if spawn point has been checked for spawns. If it has since the
     * last update, we must be in the next time step. Resets spawnedThisStep
     * when new time is discovered.
     *
     * @param spawnPoint SpawnPoint being checked by the simulator for spawning.
     * @param timeStep Time increment from last check by the simulator.
     */
    @Override
    protected void updateTimeTracking(SpawnPoint spawnPoint, double timeStep) {
        if (spawnsCheckedThisStep.contains(spawnPoint) || laststepsize != timeStep) {
            spawnsCheckedThisStep = new HashSet<SpawnPoint>();
            spawnedThisStep = new HashSet<SpawnPoint>();
            spawnsCheckedThisStep.add(spawnPoint);
            laststepsize = timeStep;
            trtime += timeStep;
        } else {
            //add element since it isn't already in set
            spawnsCheckedThisStep.add(spawnPoint);
        }
    }

    /**
     *
     */
    //protected void updateSpawnedThisStep()
}
