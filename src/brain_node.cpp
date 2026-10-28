#include <memory>
#include <chrono>
#include <atomic>
#include <algorithm>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// Parametri della Milestone v1 (Capitolo 4)
constexpr double kCruisePedal   = 0.40;   // ~15 km/h
constexpr double kBrakeMax      = -1.00;  // freno hardware massimo
constexpr double kStartDist     = 30.0;   // d_start [m]
constexpr double kStopDist      = 5.0;    // d_stop  [m]
constexpr double kJerkMax       = 0.08;   // variazione massima del pedale per tick
constexpr double kTickHz        = 30.0;
constexpr auto   kSensorTimeout = 500ms;  // telemetria piu' vecchia => fail-safe

// Telemetria condivisa tra callback ROS 2 e Behavior Tree.
// Finche' non arriva il primo messaggio vale 0 => lo shuttle resta fermo.
std::atomic<float> g_closest_obstacle{0.0f};
std::atomic<int64_t> g_last_obstacle_ns{0};

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr g_cmd_pub;
double g_last_pedal = 0.0;  // ultimo pedale pubblicato (usato solo dal thread del BT)

// Pubblica su /cmd_vel: linear.x = pedale [-1, 1], angular.z = sterzo [-1, 1]
void publishCmd(double steering, double pedal) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = pedal;
    msg.angular.z = steering;
    g_cmd_pub->publish(msg);
    g_last_pedal = pedal;
}

// Node 1: ApolloPathObstacleCheck
class ApolloPathObstacleCheck : public BT::ConditionNode {
public:
    ApolloPathObstacleCheck(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("threshold") };
    }

    BT::NodeStatus tick() override {
        double thresh = kStartDist;
        getInput("threshold", thresh);
        float current_dist = g_closest_obstacle.load();
        if (current_dist < thresh) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

// Node 2: ApolloStopTask
// Rampa di frenata quadratica p_target(d) = -((d_start - d) / (d_start - d_stop))^2
// con limitazione del Jerk. Termina con SUCCESS quando d <= d_stop (passa a
// ApolloEmergencyStop). Poiche' la Sequence di BTCPP v3 non rivaluta la condizione
// mentre questo nodo e' RUNNING, il nodo stesso verifica se la strada torna libera:
// se d >= d_start per almeno "duration" secondi restituisce FAILURE e il Fallback
// riprende la marcia di crociera.
class ApolloStopTask : public BT::StatefulActionNode {
public:
    ApolloStopTask(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("duration") };
    }

    BT::NodeStatus onStart() override {
        duration_ = 1.2;
        getInput("duration", duration_);
        pedal_ = g_last_pedal;
        clearing_ = false;
        if (g_closest_obstacle.load() <= kStopDist) {
            return BT::NodeStatus::SUCCESS;
        }
        return onRunning();
    }

    BT::NodeStatus onRunning() override {
        float dist = g_closest_obstacle.load();
        if (dist <= kStopDist) {
            return BT::NodeStatus::SUCCESS;
        }

        if (dist >= kStartDist) {
            auto now = SteadyClock::now();
            if (!clearing_) {
                clearing_ = true;
                clear_since_ = now;
            }
            if (std::chrono::duration<double>(now - clear_since_).count() >= duration_) {
                return BT::NodeStatus::FAILURE;
            }
        } else {
            clearing_ = false;
        }

        double ratio = std::clamp((kStartDist - dist) / (kStartDist - kStopDist), 0.0, 1.0);
        double target = -ratio * ratio;
        double delta = std::clamp(target - pedal_, -kJerkMax, kJerkMax);
        pedal_ += delta;
        publishCmd(0.0, pedal_);
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        clearing_ = false;
    }

private:
    double duration_ = 1.2;
    double pedal_ = 0.0;
    bool clearing_ = false;
    SteadyClock::time_point clear_since_;
};

// Node 3: ApolloEmergencyStop
class ApolloEmergencyStop : public BT::SyncActionNode {
public:
    ApolloEmergencyStop(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        publishCmd(0.0, kBrakeMax);
        return BT::NodeStatus::SUCCESS;
    }
};

// Node 4: ApolloStraightDrive
class ApolloStraightDrive : public BT::SyncActionNode {
public:
    ApolloStraightDrive(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        publishCmd(0.0, kCruisePedal);
        return BT::NodeStatus::SUCCESS;
    }
};

int64_t steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        SteadyClock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("brain_node");

    std::string default_tree;
    try {
        default_tree = ament_index_cpp::get_package_share_directory("iseauto_bt") +
                       "/bt_trees/iseauto_tree.xml";
    } catch (const std::exception&) {
        default_tree = "bt_trees/iseauto_tree.xml";
    }
    std::string tree_file = node->declare_parameter<std::string>("tree_file", default_tree);

    g_cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto obstacle_sub = node->create_subscription<std_msgs::msg::Float32>(
        "/closest_obstacle", 10,
        [](const std_msgs::msg::Float32::SharedPtr msg) {
            g_closest_obstacle.store(msg->data);
            g_last_obstacle_ns.store(steadyNowNs());
        });

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ApolloPathObstacleCheck>("ApolloPathObstacleCheck");
    factory.registerNodeType<ApolloStopTask>("ApolloStopTask");
    factory.registerNodeType<ApolloEmergencyStop>("ApolloEmergencyStop");
    factory.registerNodeType<ApolloStraightDrive>("ApolloStraightDrive");

    auto tree = factory.createTreeFromFile(tree_file);

    RCLCPP_INFO(node->get_logger(), "ISEAuto Brain Node (v1) Started - tree: %s", tree_file.c_str());

    rclcpp::Rate rate(kTickHz);
    double wcet_ms = 0.0;
    uint64_t ticks = 0;
    bool sensor_ok = false;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Watchdog: senza telemetria recente si considera un ostacolo a 0 m
        // (arresto di emergenza) invece di proseguire alla cieca.
        int64_t last = g_last_obstacle_ns.load();
        bool fresh = last != 0 &&
            (steadyNowNs() - last) < std::chrono::nanoseconds(kSensorTimeout).count();
        if (!fresh) {
            g_closest_obstacle.store(0.0f);
        }
        if (fresh != sensor_ok) {
            sensor_ok = fresh;
            if (fresh) {
                RCLCPP_INFO(node->get_logger(), "Telemetria /closest_obstacle ricevuta");
            } else {
                RCLCPP_WARN(node->get_logger(), "Telemetria assente: arresto di sicurezza");
            }
        }

        auto t0 = SteadyClock::now();
        tree.tickRoot();
        double tick_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - t0).count();
        wcet_ms = std::max(wcet_ms, tick_ms);

        if (++ticks % static_cast<uint64_t>(5 * kTickHz) == 0) {
            RCLCPP_INFO(node->get_logger(), "d=%.2f m  pedale=%.2f  WCET tick=%.3f ms",
                        g_closest_obstacle.load(), g_last_pedal, wcet_ms);
        }

        rate.sleep();
    }

    tree.haltTree();
    rclcpp::shutdown();
    return 0;
}
