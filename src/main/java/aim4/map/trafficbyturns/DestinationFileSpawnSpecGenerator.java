package aim4.map.trafficbyturns;

import aim4.config.Constants;
import aim4.config.SimConfig;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.destination.FileBasedDestinationSelector;
import aim4.map.lane.Lane;
import aim4.util.Util;
import aim4.vehicle.VehicleSpecDatabase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class DestinationFileSpawnSpecGenerator extends FileSpawnSpecGenerator {

    /**
     * Records index of last time requested
     */
    protected int lastindex;
    /**
     * Caches start time of first time slot requested, which should be 0.
     */
    private double start;
    /**
     * Map of road to list of turn counts for each time segment
     */
    private final Map<Constants.Direction, ArrayList<TurnMovementCountForRoad>> turncounts;
    /**
     * Total number of vehicles scheduled per road up to this point.
     */
    private final HashMap<Constants.Direction, Integer> totalScheduled;
    /**
     * Total number of vehicles spawned per road up to this point.
     */
    protected final HashMap<Constants.Direction, Integer> totalSpawned;
    /**
     * Map to arrays that track the times and directions for vehicles that
     * should be used when spawning
     */
    protected final HashMap<Constants.Direction, LinkedList<SpawnScheduleInformation>> spawnQueues;
    /**
     * File based destination selector. Maps action-> destination road
     */
    protected final FileBasedDestinationSelector destselect;
    /**
     * Used to keep track of the spawn points that have been checked in a single
     * time step.
     */
    protected HashSet<SpawnPoint> spawnsCheckedThisStep;
    /**
     * Used to keep track of step sizes.
     */
    protected double laststepsize;

    //FileDestination selector chosen rather than generic to enforce the ability to call act(lane, action).
    public DestinationFileSpawnSpecGenerator(Map<Constants.Direction, ArrayList<TurnMovementCountForRoad>> turncounts, FileBasedDestinationSelector fbds) {
        this.turncounts = turncounts;
        destselect = fbds;

        //initialize turns at present, totals, and next spawns for every road
        totalScheduled = new HashMap<Constants.Direction, Integer>();
        totalSpawned = new HashMap<Constants.Direction, Integer>();
        spawnQueues = new HashMap<Constants.Direction, LinkedList<SpawnScheduleInformation>>();
        start = 0;
        for (Constants.Direction key : turncounts.keySet()) {
            totalScheduled.put(key, 0);
            totalSpawned.put(key, 0);
            spawnQueues.put(key, new LinkedList<SpawnScheduleInformation>());
            HashMap<Constants.TurnDirection, Integer> imthemap = new HashMap<Constants.TurnDirection, Integer>();
            for (Constants.TurnDirection act : Constants.TurnDirection.values()) {
                imthemap.put(act, 0);
            }
        }

        spawnsCheckedThisStep = new HashSet<SpawnPoint>();
        laststepsize = 0;
        trtime = 0;
        lastindex = -1;
    }

    /////////////////////////////////
    // PUBLIC FIELDS
    /////////////////////////////////
    //THIS ONLY FUNCTIONS WITH 1 INTERSECTION BECAUSE OF ** BELOW, also requires that the lane goes all the way to the intersection from spawn todo!!
    @Override
    public List<SpawnPoint.SpawnSpec> act(SpawnPoint spawnPoint, double timeStep, SimConfig.VEHICLE_TYPE vehicleType) {
        updateTimeTracking(spawnPoint, timeStep);
        ArrayList<SpawnPoint.SpawnSpec> specs = new ArrayList<SpawnPoint.SpawnSpec>();
        Road spawnroad = spawnPoint.getLane().getContainingRoad();
        //** THIS ONLY FUNCTIONS WITH 1 INTERSECTION
        Lane spawnLane = spawnPoint.getLane();
        Constants.Direction roadDir = Util.getDirectionFromHeadingCardinalOrIntercardinal(spawnLane.getLaneIM().firstIntersectionManager().getIntersection().getEntryHeading(spawnLane));
        SpawnScheduleInformation ssi = shouldSpawnVehicle(roadDir, trtime);
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
    protected SpawnScheduleInformation shouldSpawnVehicle(Constants.Direction roadDir, double time) {
        int indx = getIndexWithTime(time);
        if (indx > lastindex) {
            updateToPresent(indx);
        }

        return getAction(roadDir, time);
    }

    protected SpawnScheduleInformation getAction(Constants.Direction roadDir, double time) {
        //shortcircuiting to check if appropriate time has passed
        /*if there are spawns left for the road, and the appropriate time has passed for a spawn, signal to spawn something.*/
        if (spawnQueues.get(roadDir).size() > 0
                && spawnQueues.get(roadDir).get(0) != null && time >= spawnQueues.get(roadDir).get(0).getSpawnTime()) {
            //any action is permitted from anywhere, so just grab the first element.
            return spawnQueues.get(roadDir).get(0);
        }
        //no spawns left or appropriate time has not passed
        return null;
    }

    protected void updateAppropriateSpawnQueue(Constants.Direction roadDir, SpawnScheduleInformation spawnSchedInfo) {
        //Also, todo AP not thread safe
        spawnQueues.get(roadDir).remove(spawnSchedInfo);
    }

    //todo!! break this up into more functions
    //updates current counts for each road of cars that need to spawn with a certain action
    void updateToPresent(int indx) {
        if (indx > lastindex) {
            //assumes all roads have the same number of time entries with same times
            //todo, should just keep the length value as a member rather that grabbing it from a list
            ArrayList<TurnMovementCountForRoad> tempal = turncounts.get((Constants.Direction) turncounts.keySet().toArray()[0]);
            //ensures we don't step over the end of the list....todo, causes problems if given negative numbers, should throw exception
            int tempindx = Math.min(indx, tempal.size() - 1);

            //update for every time index from last known/requested to now
            boolean timeset = false;
            for (int i = lastindex + 1; i < tempal.size() && i <= indx; i++) {
                for (Constants.Direction roadidentifier : turncounts.keySet()) {
                    //todo!! AP this isn't thread safe
                    LinkedList<SpawnScheduleInformation> queueForRoad = spawnQueues.get(roadidentifier);
                    TurnMovementCountForRoad turns = turncounts.get(roadidentifier).get(i);
                    if (!timeset) {
                        start = turns.getTimeOffset();
                        timeset = true;
                    } else if (turns.getTimeOffset() != start) {
                        throw new RuntimeException("Start times for interval for turns of equivilent index do not match: " + start + " doesn't match " + turns.getTimeInterval());
                    }

                    int totalfromseg = 0;
                    int heldOverTotal = queueForRoad.size();
                    //holds actions that need to be associated with spawns in next time step
                    EnumMap<Constants.TurnDirection, Integer> nextStepActionPool = new EnumMap<Constants.TurnDirection, Integer>(Constants.TurnDirection.class);

                    //count actions possible for this road
                    for (Constants.TurnDirection act : turns.getActions()) {
                        int addStep = turns.getFromAction(act);
                        totalfromseg += addStep;
                        if (nextStepActionPool.containsKey(act)) {
                            throw new RuntimeException("Duplicate turn direction found when moving to next time slot in DestinationFileSpawnSpecGenerator: " + act.name());
                        } else if (addStep > 0) {
                            nextStepActionPool.put(act, addStep);
                        }
                        //record total that will be scheduled
                        totalScheduled.put(roadidentifier, totalScheduled.get(roadidentifier) + addStep);
                    }
                    totalfromseg += heldOverTotal;

                    if (totalfromseg > 0) {
                        //preserve information about cars that didn't get sent in last interval because of timestep being too large
                        LinkedList<SpawnScheduleInformation> schedTimes = new LinkedList<SpawnScheduleInformation>(queueForRoad);
                        Random rand = Util.RANDOM_NUM_GEN;
                        double gap = 0;
                        double[] spawnTimes = new double[totalfromseg - heldOverTotal];
                        for (int j = 0; j < spawnTimes.length; j++) {
                            spawnTimes[j] = turns.getTimeOffset() + (turns.getTimeInterval() - gap) * rand.nextDouble();
                        }
                        Arrays.sort(spawnTimes);

                        for (int timeindx = heldOverTotal; timeindx < totalfromseg; timeindx++) {
                            //randomly select arrival time in interval
                            Constants.TurnDirection actFromPool = null;
                            int selection = Util.RANDOM_NUM_GEN.nextInt(nextStepActionPool.keySet().size());
                            for (Constants.TurnDirection actToInspect : nextStepActionPool.keySet()) {
                                if (selection == 0) {
                                    actFromPool = actToInspect;
                                    //adjust count of actions that need to appear in next time step, assuming this action was chosen
                                    if (nextStepActionPool.get(actToInspect) == 1) {
                                        nextStepActionPool.remove(actToInspect);
                                    } else {
                                        nextStepActionPool.put(actToInspect, nextStepActionPool.get(actToInspect) - 1);
                                    }
                                    break;
                                }
                                //count down. When selection is 0, the element we're on is the one we want to choose.
                                selection--;
                            }
                            if (actFromPool == null) {
                                throw new RuntimeException("actFromPool in DestinationFileSpawnSpecGenerator was null, even though an action should have been selected because actions were provided.");
                            }
                            schedTimes.add(new SpawnScheduleInformation(spawnTimes[timeindx - heldOverTotal], actFromPool));
                        }

                        spawnQueues.put(roadidentifier, schedTimes);
                    } else {
                        //reset queue
                        spawnQueues.put(roadidentifier, new LinkedList<SpawnScheduleInformation>());
                    }
                }
            }
            lastindex = tempindx;

            //error checking
            if (getTotalVehiclesScheduled() - getTotalVehiclesSpawned() != getVehiclesLeftInCurrentTimeSlot()) {
                throw new RuntimeException("Number of vehicles scheduled minus those spawned does not meet the number left in the current slot: " + getTotalVehiclesScheduled() + " - " + getTotalVehiclesSpawned() + " != " + getVehiclesLeftInCurrentTimeSlot());
            }
        }
    }

    protected int getIndexWithTime(double time) {
        //assumes all lists are same size and have same steps
        if (turncounts.keySet().size() > 0) {
            ArrayList<TurnMovementCountForRoad> al = turncounts.get((Constants.Direction) turncounts.keySet().toArray()[0]);
            return binarySearchTime(Math.max(0, lastindex), al.size() - 1, time, al);
        }
        return -2;
    }

    private int binarySearchTime(int lef, int righ, double time, ArrayList<TurnMovementCountForRoad> lis) {
        int left = lef;
        int right = righ;

        //invalid parameter check
        if (left < 0 || left >= lis.size() || right < 0 || right >= lis.size()) {
            return -1;
        }

        int mid;
        double midval;
        while (left <= right) {
            mid = (left + right) / 2;
            midval = lis.get(mid).getTimeOffset();
            if (time >= midval) {
                //check if time falls in interval, if not, move the mark.
                if (time < lis.get(mid).getEndTime()) {
                    return mid;
                }

                left = mid + 1;
            } else {
                right = mid - 1;
            }
        }
        return -1;
    }

    /**
     * Check if spawn point has been checked for spawns. If it has since the
     * last update, we must be in the next time step.
     *
     * @param spawnPoint SpawnPoint being checked by the simulator for spawning.
     * @param timeStep Time increment from last check by the simulator.
     */
    protected void updateTimeTracking(SpawnPoint spawnPoint, double timeStep) {
        if (spawnsCheckedThisStep.contains(spawnPoint) || laststepsize != timeStep) {
            spawnsCheckedThisStep = new HashSet<SpawnPoint>();
            spawnsCheckedThisStep.add(spawnPoint);
            laststepsize = timeStep;
            trtime += timeStep;
        } else {
            //add element since it isn't already in set
            spawnsCheckedThisStep.add(spawnPoint);
        }
    }

    @Override
    public void vehicleGenerated() {
        //throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public List<SpawnPoint.SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
        return act(spawnPoint, timeStep, SimConfig.VEHICLE_TYPE.AUTO);
    }

    @Override
    public int getTotalVehiclesSpawned() {
        int totalSpawn = 0;
        for (Constants.Direction roadDir : totalSpawned.keySet()) {
            int val = totalSpawned.get(roadDir);
            if (val >= 0) {
                totalSpawn += val;
            } else {
                throw new RuntimeException("Negative number of vehicles spawned in FileSpawnSpecGenerator for road heading: " + roadDir.name());
            }
        }
        return totalSpawn;
    }

    @Override
    public int getTotalVehiclesScheduled() {
        int totalSched = 0;
        for (Constants.Direction roadDir : totalScheduled.keySet()) {
            int val = totalScheduled.get(roadDir);
            if (val >= 0) {
                totalSched += val;
            } else {
                throw new RuntimeException("Negative number of vehicles scheduled in FileSpawnSpecGenerator for road heading: " + roadDir.name());
            }
        }
        return totalSched;
    }

    @Override
    public int getVehiclesLeftInCurrentTimeSlot() {
        int totalLeft = 0;
        for (Constants.Direction roadDir : spawnQueues.keySet()) {
            int val = spawnQueues.get(roadDir).size();
            if (val >= 0) {
                totalLeft += val;
            } else {
                throw new RuntimeException("Negative number of vehicles left in time slot in FileSpawnSpecGenerator for road heading: " + roadDir.name());
            }
        }
        return totalLeft;
    }
}
