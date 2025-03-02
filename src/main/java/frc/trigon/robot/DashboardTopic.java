package frc.trigon.robot;

import edu.wpi.first.networktables.*;

import java.io.Closeable;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class DashboardTopic implements Closeable {
    private static final String
            DASHBOARD_TABLE = "Dashboard",
            ROBOT_TABLE_PATH = "robot",
            DASHBOARD_TABLE_PATH = "dashboard";
    private static final Set<DashboardTopic> topics = new HashSet<>();

    private final Consumer<NetworkTableValue> onChanged;
    private final GenericPublisher pub;
    private final GenericSubscriber sub;
    private final AtomicReference<NetworkTableValue> value;
    private static final NetworkTable dashboardTable = NetworkTableInstance.getDefault().getTable(DASHBOARD_TABLE);

    public DashboardTopic(String name, NetworkTableValue defaultValue, Consumer<NetworkTableValue> onChanged) {
        sub = dashboardTable.getSubTable(DASHBOARD_TABLE_PATH).getTopic(name).genericSubscribe(PubSubOption.keepDuplicates(true));
        pub = dashboardTable.getSubTable(ROBOT_TABLE_PATH).getTopic(name).genericPublish(defaultValue.getType().getValueStr());
        this.value = new AtomicReference<>();
        set(defaultValue);

        topics.add(this);

        this.onChanged = (val) -> {
            if(val.getType() == NetworkTableType.kUnassigned) return;
            set(val);
            onChanged.accept(val);
        };
    }

    public void set(NetworkTableValue value) {
        pub.set(value);
        this.value.set(value);
    }

    public NetworkTableValue get() {
        return value.get();
    }

    public void update() {
        Arrays.stream(sub.readQueue()).reduce((a, b) -> b).ifPresent(onChanged);
    }

    public static void updateAll() {
        for(DashboardTopic topic : new HashSet<>(topics))
            topic.update();
    }

    @Override
    public void close() {
        pub.close();
        sub.close();
        topics.remove(this);
    }
}
