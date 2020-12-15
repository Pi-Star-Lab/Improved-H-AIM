package aim4.config.ringbarrier;

import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

public class MaximumGreenTable {

    //table of volume per lane per hour per phase (veh/hr/ln) by cycle length to maximum, green time
    private static NavigableMap<Double, NavigableMap<Double, Double>> table = null;

    public static Double getMaximumGreenTimeInSecondsClosestToProvidedKeys(double phaseVolumePerHourPerLane, double cycleLength) {
        initializeTableIfNeeded();
        Entry<Double, NavigableMap<Double, Double>> preferredOuterEntry = getPreferredOuterEntry(phaseVolumePerHourPerLane, table.floorEntry(phaseVolumePerHourPerLane), table.ceilingEntry(phaseVolumePerHourPerLane));
        if (preferredOuterEntry.getValue() != null) {
            Double val = getPreferredInnerEntry(cycleLength, preferredOuterEntry.getValue().floorEntry(cycleLength), preferredOuterEntry.getValue().ceilingEntry(cycleLength)).getValue();
            if (val == null) {
                throw new RuntimeException("Inner result is null. This is not allowed.");
            }
            return val;
        } else {
            throw new RuntimeException("Outer map is null. This is not allowed.");
        }
    }

    private static Entry<Double, NavigableMap<Double, Double>> getPreferredOuterEntry(double keyval, Entry<Double, NavigableMap<Double, Double>> entry1, Entry<Double, NavigableMap<Double, Double>> entry2) {
        if (entry1 == null || entry2 == null) {
            if (entry1 == null && entry2 != null) {
                return entry2;
            } else if (entry1 != null && entry2 == null) {

                return entry1;
            } else if (entry1 == null && entry2 == null) {
                throw new RuntimeException("Both entries are null. This is not allowed.");
            }
        }

        if (entry1.getKey() == null) {
            throw new RuntimeException("Entry1's key is null. This is not allowed.");
        } else if (entry2.getKey() == null) {
            throw new RuntimeException("Entry2's key is null. This is not allowed.");
        }

        if (Math.abs(entry1.getKey() - keyval) <= Math.abs(entry2.getKey() - keyval)) {
            return entry1;
        } else { // (Math.abs(entry1.getKey() - keyval) > Math.abs(entry2.getKey() - keyval)) {
            return entry2;
        }
    }

    private static Entry<Double, Double> getPreferredInnerEntry(double keyval, Entry<Double, Double> entry1, Entry<Double, Double> entry2) {
        if (entry1 == null || entry2 == null) {
            if (entry1 == null && entry2 != null) {
                return entry2;
            } else if (entry1 != null && entry2 == null) {

                return entry1;
            } else if (entry1 == null && entry2 == null) {
                throw new RuntimeException("Both entries are null. This is not allowed.");
            }
        }

        if (entry1.getKey() == null) {
            throw new RuntimeException("Entry1's key is null. This is not allowed.");
        } else if (entry2.getKey() == null) {
            throw new RuntimeException("Entry2's key is null. This is not allowed.");
        }

        if (Math.abs(entry1.getKey() - keyval) <= Math.abs(entry2.getKey() - keyval)) {
            return entry1;
        } else { // (Math.abs(entry1.getKey() - keyval) > Math.abs(entry2.getKey() - keyval)) {
            return entry2;
        }
    }

    private static void initializeTableIfNeeded() {
        if (table == null) {
            table = new TreeMap<Double, NavigableMap<Double, Double>>();
            //table 5-6 https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter5.htm
            table.put(100.0, new TreeMap<Double, Double>());
            table.get(100.0).put(50.0, 15.0);
            table.get(100.0).put(60.0, 15.0);
            table.get(100.0).put(70.0, 15.0);
            table.get(100.0).put(80.0, 15.0);
            table.get(100.0).put(90.0, 15.0);
            table.get(100.0).put(100.0, 15.0);
            table.get(100.0).put(110.0, 15.0);
            table.get(100.0).put(120.0, 15.0);

            table.put(200.0, new TreeMap<Double, Double>());
            table.get(200.0).put(50.0, 15.0);
            table.get(200.0).put(60.0, 15.0);
            table.get(200.0).put(70.0, 15.0);
            table.get(200.0).put(80.0, 15.0);
            table.get(200.0).put(90.0, 16.0);
            table.get(200.0).put(100.0, 18.0);
            table.get(200.0).put(110.0, 19.0);
            table.get(200.0).put(120.0, 21.0);

            table.put(300.0, new TreeMap<Double, Double>());
            table.get(300.0).put(50.0, 15.0);
            table.get(300.0).put(60.0, 16.0);
            table.get(300.0).put(70.0, 19.0);
            table.get(300.0).put(80.0, 21.0);
            table.get(300.0).put(90.0, 24.0);
            table.get(300.0).put(100.0, 26.0);
            table.get(300.0).put(110.0, 29.0);
            table.get(300.0).put(120.0, 31.0);

            table.put(400.0, new TreeMap<Double, Double>());
            table.get(400.0).put(50.0, 18.0);
            table.get(400.0).put(60.0, 21.0);
            table.get(400.0).put(70.0, 24.0);
            table.get(400.0).put(80.0, 28.0);
            table.get(400.0).put(90.0, 31.0);
            table.get(400.0).put(100.0, 34.0);
            table.get(400.0).put(110.0, 38.0);
            table.get(400.0).put(120.0, 41.0);

            table.put(500.0, new TreeMap<Double, Double>());
            table.get(500.0).put(50.0, 22.0);
            table.get(500.0).put(60.0, 26.0);
            table.get(500.0).put(70.0, 30.0);
            table.get(500.0).put(80.0, 34.0);
            table.get(500.0).put(90.0, 39.0);
            table.get(500.0).put(100.0, 43.0);
            table.get(500.0).put(110.0, 47.0);
            table.get(500.0).put(120.0, 51.0);

            table.put(600.0, new TreeMap<Double, Double>());
            table.get(600.0).put(50.0, 26.0);
            table.get(600.0).put(60.0, 31.0);
            table.get(600.0).put(70.0, 36.0);
            table.get(600.0).put(80.0, 41.0);
            table.get(600.0).put(90.0, 46.0);
            table.get(600.0).put(100.0, 51.0);
            table.get(600.0).put(110.0, 56.0);
            table.get(600.0).put(120.0, 61.0);

            table.put(700.0, new TreeMap<Double, Double>());
            table.get(700.0).put(50.0, 30.0);
            table.get(700.0).put(60.0, 36.0);
            table.get(700.0).put(70.0, 42.0);
            table.get(700.0).put(80.0, 48.0);
            table.get(700.0).put(90.0, 54.0);
            table.get(700.0).put(100.0, 59.0);
            table.get(700.0).put(110.0, 65.0);
            table.get(700.0).put(120.0, 71.0);

            table.put(800.0, new TreeMap<Double, Double>());
            table.get(800.0).put(50.0, 34.0);
            table.get(800.0).put(60.0, 41.0);
            table.get(800.0).put(70.0, 48.0);
            table.get(800.0).put(80.0, 54.0);
            table.get(800.0).put(90.0, 61.0);
            table.get(800.0).put(100.0, 68.0);
            table.get(800.0).put(110.0, 74.0);
            table.get(800.0).put(120.0, 81.0);
        }
    }
}
