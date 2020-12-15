package aim4.map.trafficbyturns;

import aim4.config.Constants;
import aim4.map.actionmapping.ActionMappingFactory;
import aim4.map.destination.FileBasedDestinationSelector;
import aim4.util.Util;
import java.io.File;
import java.io.FileNotFoundException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class TrafficFlowReaderFactory {

    private static final double DAY = 86400;
    private static final String DELIM = ",";

    //todo!! this needs to be split into functions
    public static TurnMovements getMovementsFromFile(File fi, Map<String, Constants.TurnDirection> labeltoaction) {
        HashMap<Constants.Direction, ArrayList<TurnMovementCountForRoad>> countlists = new HashMap<Constants.Direction, ArrayList<TurnMovementCountForRoad>>();
        Scanner in = null;
        try {
            in = new Scanner(fi);
        } catch (FileNotFoundException ex) {
            throw new RuntimeException(ex);
        }

        //get road names
        String[] roadStrings = in.nextLine().trim().split(DELIM);
        Constants.Direction[] roads = new Constants.Direction[roadStrings.length];
        for (int i = 0; i < roads.length; i++) {
            roads[i] = Util.getParseEnumFromStringCaseInsen(roadStrings[i], Constants.Direction.class);
            countlists.put(roads[i], new ArrayList<TurnMovementCountForRoad>());
        }

        //handle header row for columns
        //todo, no way to map total keyword, need to fix that
        ArrayList<ArrayList<Constants.TurnDirection>> elemsperroad = new ArrayList<ArrayList<Constants.TurnDirection>>();
        String[] templine = in.nextLine().trim().split(DELIM);
        //read and track the directions for each road
        for (int lineelemindex = 0, roadindx = 0; lineelemindex < templine.length && roadindx < roads.length; lineelemindex++) {
            String elem = templine[lineelemindex];
            if (elem.equals("Total")) {
                roadindx++;
            } else {
                if (elemsperroad.size() == roadindx) {
                    elemsperroad.add(new ArrayList<Constants.TurnDirection>());
                }
                if (labeltoaction.containsKey(elem)) {
                    elemsperroad.get(roadindx).add(labeltoaction.get(elem));
                } else {
                    throw new RuntimeException("Invalid/unmapped turn direction found while parsing file: " + elem);
                }
            }
        }

        //handle parsing individual rows
        boolean inittime = false;
        long starttime = 0;
        long numtime;
        double offsettime = Double.NaN;
        double prevoffsettime;
        double initialdiff = Double.POSITIVE_INFINITY;
        int total = 0;
        //used to confirm inference that the last time step should be of the same size as all the previous ones by checking to see that all of them were the same
        boolean lastsame = true;
        int daymult = 0;
        boolean wrap = false;
        TurnMovementCountForRoad[] prevcounts = null;

        for (int row = 0; in.hasNextLine(); row++) {
            templine = in.nextLine().trim().split(DELIM);
            try {
                //get time offset
                numtime = (new java.text.SimpleDateFormat("h:mm a")).parse(templine[0]).getTime();
            } catch (ParseException ex) {
                throw new RuntimeException("Error parsing time for row in traffic file, expected: \"" + templine[0] + "\"");
            }

            //init original time if it isn't set
            if (!inittime) {
                starttime = numtime;
                inittime = true;
            }

            //handle if data spans across multiple days
            if (numtime < starttime && !wrap) {
                wrap = true;
                daymult++;
            } else if (numtime >= starttime && wrap) {
                wrap = false;
            }

            //prepping to split row information by road
            TurnMovementCountForRoad[] tmcfrs = new TurnMovementCountForRoad[roads.length];
            prevoffsettime = offsettime;
            offsettime = (numtime - starttime) / 1000 + daymult * DAY;
            for (int i = 0; i < tmcfrs.length; i++) {
                tmcfrs[i] = new TurnMovementCountForRoad(offsettime);
            }

            //set time on previous row if applicable
            for (int i = 0; prevcounts != null && i < prevcounts.length; i++) {
                prevcounts[i].setTimeInterval(offsettime - prevcounts[i].getTimeOffset());
            }

            //handle recording initialdiff and checking if timing is consistent across all rows
            //todo major priority, make this more robust...it's using doubles which could easily croak with slight errors
            if (row == 1) {
                initialdiff = offsettime;
            } else if (row > 0 && initialdiff != offsettime - prevoffsettime) {
                lastsame = false;
            }

            //handle row items, assuming actions for one road are followed by a total field
            for (int lineelemindex = 1, roadindx = 0, numelem = 0; lineelemindex < templine.length && roadindx < roads.length; lineelemindex++) {
                String elem = templine[lineelemindex];
                //Record count of appropriate action for appropriate row and road (column)
                if (numelem < elemsperroad.get(roadindx).size()) {
                    if (!tmcfrs[roadindx].putActValIfAbsent(elemsperroad.get(roadindx).get(numelem), Integer.parseInt(elem))) {
                        throw new RuntimeException("Problem when parsing turn movements file. Duplicate column: " + elemsperroad.get(roadindx).get(numelem).name());
                    }
                    numelem++;
                }

                //actually save/copy the values into the map of lists that will be returned
                if (numelem >= elemsperroad.get(roadindx).size()) {
                    //add turn move count to array list representing column for road
                    countlists.get(roads[roadindx]).add(tmcfrs[roadindx]);
                    total += tmcfrs[roadindx].getTotal();
                    numelem = 0;
                    roadindx++;
                    lineelemindex++; //skip "total"
                }
            }
            prevcounts = tmcfrs;
        }
        in.close();

        if (lastsame) {
            for (int i = 0; prevcounts != null && i < prevcounts.length; i++) {
                prevcounts[i].setTimeInterval(initialdiff);
            }
        }

        return new TurnMovements(countlists, total);
    }

    public static TurnMovements getMovementsFromFileDefaultMapping(File fi) {
        return getMovementsFromFile(fi, ActionMappingFactory.getDefaultActionMapping());
    }

    public static DestinationFileSpawnSpecGenerator getDestFileSpawnSpecGenFromFile(File fi, Map<String, Constants.TurnDirection> labeltoaction, FileBasedDestinationSelector destselect) {
        return new DestinationFileSpawnSpecGenerator(getMovementsFromFile(fi, labeltoaction).getCountLists(), destselect);
    }

    public static LaneRestrictedFileSpawnSpecGenerator getLaneRestrDestFileSpawnSpecGenFromFile(File fi, Map<String, Constants.TurnDirection> labeltoaction, FileBasedDestinationSelector destselect) {
        return new LaneRestrictedFileSpawnSpecGenerator(getMovementsFromFile(fi, labeltoaction).getCountLists(), destselect);
    }

    public static DestinationFileSpawnSpecGenerator getDestFileSpawnSpecGenFromFileDefaultMapping(File fi, FileBasedDestinationSelector destselect) {
        return new DestinationFileSpawnSpecGenerator(getMovementsFromFile(fi, ActionMappingFactory.getDefaultActionMapping()).getCountLists(), destselect);
    }

    public static LaneRestrictedFileSpawnSpecGenerator getLaneRestrDestFileSpawnSpecGenFromFileDefaultMapping(File fi, FileBasedDestinationSelector destselect) {
        return new LaneRestrictedFileSpawnSpecGenerator(getMovementsFromFile(fi, ActionMappingFactory.getDefaultActionMapping()).getCountLists(), destselect);
    }

    public static LaneRestrictedFileSpawnSpecGenerator getLaneRestrDestFileSpawnSpecGen(Map<Constants.Direction, ArrayList<TurnMovementCountForRoad>> turncounts, FileBasedDestinationSelector destselect) {
        return new LaneRestrictedFileSpawnSpecGenerator(turncounts, destselect);
    }

    public static DestinationFileSpawnSpecGenerator getDestFileSpawnSpecGen(Map<Constants.Direction, ArrayList<TurnMovementCountForRoad>> turncounts, FileBasedDestinationSelector destselect) {
        return new DestinationFileSpawnSpecGenerator(turncounts, destselect);
    }
}
