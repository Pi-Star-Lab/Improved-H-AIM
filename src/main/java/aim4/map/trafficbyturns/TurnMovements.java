package aim4.map.trafficbyturns;

import aim4.config.Constants;
import java.util.ArrayList;
import java.util.HashMap;

public class TurnMovements {

    /**
     * TurnMovementCountForRoads (contains: start time for row, duration, number
     * of turns per action action all for a specific road)
     */
    private final HashMap<Constants.Direction, ArrayList<TurnMovementCountForRoad>> countlists;
    /**
     * Cached size of lists in countlists
     */
    private final int listSize;
    /**
     * total of turns across all the TurnMovementCountForRoads
     */
    private final int total;
    /**
     * total number of turns expected up to a given time slot
     */
    private final int[] beforeSlotTotals;
    /**
     * total number of turns expected up to and including a given time slot
     */
    private final int[] includingSlotTotals;

    //todo split this constructor up
    public TurnMovements(HashMap<Constants.Direction, ArrayList<TurnMovementCountForRoad>> countlists, int total) {
        int tempListSize = -1;
        Constants.Direction firstDir = null;
        for (Constants.Direction dir : countlists.keySet()) {
            if (tempListSize < 0) {
                tempListSize = countlists.get(dir).size();
                firstDir = dir;
            }

            if (tempListSize != countlists.get(dir).size()) {
                throw new RuntimeException("TurnMovementCountForRoad lists have different sizes. Error discovered at: " + firstDir.name() + " with length " + tempListSize + " and " + dir.name() + " with length " + countlists.get(dir).size() + ".");
            }
        }
        listSize = tempListSize;
        beforeSlotTotals = new int[listSize];
        includingSlotTotals = new int[listSize];

        firstDir = null;
        int upToTotal = 0;
        int includingTotal = 0;
        for (int i = 0; i < listSize; i++) {
            double start = -1;
            double duration = -1;
            for (Constants.Direction dir : countlists.keySet()) {
                if (start < 1) {
                    start = countlists.get(dir).get(i).getTimeOffset();
                    duration = countlists.get(dir).get(i).getTimeInterval();
                    firstDir = dir;
                }

                if (start != countlists.get(dir).get(i).getTimeOffset() || duration != countlists.get(dir).get(i).getTimeInterval()) {
                    throw new RuntimeException("TurnMovementCountForRoad lists are not homogeneous in terms of start times and time intervals at same index (" + i + "):\n"
                            + firstDir.name() + " start: " + start + " duration: " + duration + "\n"
                            + dir.name() + " start: " + countlists.get(dir).get(i).getTimeOffset() + " duration: " + countlists.get(dir).get(i).getTimeInterval());
                } else {
                    if (i > 0) {
                        upToTotal += countlists.get(dir).get(i - 1).getTotal();
                    }
                    includingTotal += countlists.get(dir).get(i).getTotal();

                    beforeSlotTotals[i] = upToTotal;
                    includingSlotTotals[i] = includingTotal;
                }
            }
        }

        this.countlists = countlists;
        this.total = total;
    }

    public HashMap<Constants.Direction, ArrayList<TurnMovementCountForRoad>> getCountLists() {
        return countlists;
    }

    public int getTotal() {
        return total;
    }

    public int getExpectedSpawnsUpToEndOfTimeSlot(double time) {
        //doesn't support a negative simulation time
        if (countlists.keySet().size() > 0 && time >= 0) {
            //getting one list of timeslots, don't care about the direction as I'm  just using it to search times.
            ArrayList<TurnMovementCountForRoad> al = countlists.get((Constants.Direction) countlists.keySet().toArray()[0]);
            if (al != null && al.size() > 0 && time > al.get(al.size() - 1).getEndTime()) {
                return includingSlotTotals[includingSlotTotals.length - 1];
            }
            int index = binarySearchTime(0, listSize - 1, time, al);
            if (index != -1) {
                return includingSlotTotals[index];
            }
        }
        return 0;
    }

    public int getExpectedSpawnsUpToBeginningOfTimeSlot(double time) {
        //doesn't support a negative simulation time
        if (countlists.keySet().size() > 0 && time >= 0) {
            //getting one list of timeslots, don't care about the direction as I'm  just using it to search times.
            ArrayList<TurnMovementCountForRoad> al = countlists.get((Constants.Direction) countlists.keySet().toArray()[0]);
            if (al != null && al.size() > 0 && time > al.get(al.size() - 1).getEndTime()) {
                return beforeSlotTotals[beforeSlotTotals.length - 1];
            }
            int index = binarySearchTime(0, listSize - 1, time, al);
            if (index != -1) {
                return beforeSlotTotals[index];
            }
        }
        return 0;
    }

    private int binarySearchTime(int lef, int righ, double time, ArrayList<TurnMovementCountForRoad> lis) {
        int left = lef;
        int right = righ;

        //invalid parameter check
        if (lis == null || left < 0 || left >= lis.size() || right < 0 || right >= lis.size()) {
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

}
