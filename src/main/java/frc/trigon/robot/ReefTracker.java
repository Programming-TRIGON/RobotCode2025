package frc.trigon.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.trigon.robot.constants.FieldConstants;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

public class ReefTracker {
    private static final Map<Face, Map<Level, Map<FieldConstants.ReefSide, DashboardTopic>>> reefMap;
    private static final Map<Face, Pair<DashboardTopic, AtomicBoolean>> algae;

    static final BranchState DEFAULT_BRANCH_STATE = BranchState.FREE;
    static final boolean DEFAULT_ALGAE_STATE = true;

    static {
        reefMap = new HashMap<>();
        for (Face face : Face.values()) {
            Map<Level, Map<FieldConstants.ReefSide, DashboardTopic>> levelMap = new HashMap<>();
            for (Level level : Level.values()) {
                Map<FieldConstants.ReefSide, DashboardTopic> sideMap = new HashMap<>();
                for (FieldConstants.ReefSide side : FieldConstants.ReefSide.values()) {
                    sideMap.put(side, makeBranchTopic(Branch.of(face, level, side)));
                }
                levelMap.put(level, sideMap);
            }
            reefMap.put(face, levelMap);
        }

        algae = new HashMap<>();
        for (Face face : Face.values()) {
            algae.put(face, new Pair<>(makeAlgaeTopic(face), new AtomicBoolean(DEFAULT_ALGAE_STATE)));
        }
    }

    public static void init() {
    }

    private static DashboardTopic getBranchTopic(Branch branch) {
        return reefMap.get(branch.face).get(branch.level).get(branch.side);
    }

    private static BranchState getBranchState(Branch branch) {
        var ordinal = Math.min(Math.max((int) getBranchTopic(branch).get().getInteger(), 0), BranchState.values().length - 1);
        return BranchState.values()[ordinal];
    }


    private static void setBranchState(Branch branch, BranchState state) {
        getBranchTopic(branch).set(NetworkTableValue.makeInteger(state.ordinal()));
    }

    private static void updateBranchFromNT(Branch branch, NetworkTableValue value) {
        if (value.getType() == NetworkTableType.kInteger) {
        }
    }

    private static DashboardTopic makeBranchTopic(Branch branch) {
        return new DashboardTopic(
                String.format("Reef/%s/%s/%s", branch.face, branch.level, branch.side), NetworkTableValue.makeInteger(ReefTracker.DEFAULT_BRANCH_STATE.ordinal()), (value) -> updateBranchFromNT(branch, value)
        );
    }

    private static Pair<DashboardTopic, AtomicBoolean> getAlgaePair(Face face) {
        return algae.get(face);
    }

    private static boolean getAlgae(Face face) {
        return getAlgaePair(face).getSecond().get();
    }

    private static void updateAlgaeCache(Face face, boolean value) {
        getAlgaePair(face).getSecond().set(value);
    }

    private static void setAlgae(Face face, boolean value) {
        getAlgaePair(face).getFirst().set(NetworkTableValue.makeBoolean(value));
        updateAlgaeCache(face, value);
    }

    private static DashboardTopic makeAlgaeTopic(Face face) {
        return new DashboardTopic(
                String.format("Reef/%s/Algae", face), NetworkTableValue.makeBoolean(ReefTracker.DEFAULT_ALGAE_STATE), (value) -> {
            if (value.getType() == NetworkTableType.kBoolean)
                getAlgaePair(face).getSecond().set(value.getBoolean());
        }
        );
    }

    public enum Face {
        A, B, C, D, E, F;

        public boolean hasAlgae() {
            return ReefTracker.getAlgae(this);
        }

        public void setAlgae(boolean value) {
            ReefTracker.setAlgae(this, value);
        }

        private Rotation2d calculateClockAngle() {
            return FieldConstants.REEF_CLOCK_POSITION_DIFFERENCE.times(-ordinal());
        }
    }

    public enum Level {
        L2, L3, L4
    }

    public static class Branch {
        public final Face face;
        public final Level level;
        public final FieldConstants.ReefSide side;

        private Branch(Face face, Level level, FieldConstants.ReefSide side) {
            this.face = face;
            this.level = level;
            this.side = side;
        }

        public static Branch of(Face face, Level level, FieldConstants.ReefSide side) {
            return new Branch(face, level, side);
        }

        public BranchState getState() {
            return ReefTracker.getBranchState(this);
        }

        public void setState(BranchState state) {
            ReefTracker.setBranchState(this, state);
        }
    }

    public enum BranchState {
        FREE, OCCUPIED, DISABLED
    }
}