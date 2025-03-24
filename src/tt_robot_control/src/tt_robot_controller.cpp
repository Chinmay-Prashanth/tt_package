#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Python.h>
#include <string>
#include <vector>
#include <cmath>

class TTRobotController : public rclcpp::Node
{
public:
    TTRobotController() : Node("tt_robot_controller")
    {
        // Initialize Python interpreter
        Py_Initialize();
        
        // Add the Python modules to the path
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("sys.path.append('/home/upsurge/tt_package/src/tt_robot_control/src')");
        
        // Import the Python modules
        pNemaModule = PyImport_ImportModule("nema");
        pServoModule = PyImport_ImportModule("SMS_STS.TTRobot.servo_control");
        
        if (!pNemaModule || !pServoModule) {
            RCLCPP_ERROR(this->get_logger(), "Failed to import Python modules");
            return;
        }
        
        // Get the Python functions
        pNemaInit = PyObject_GetAttrString(pNemaModule, "init_motors");
        pNemaMove = PyObject_GetAttrString(pNemaModule, "move_motors");
        pServoInit = PyObject_GetAttrString(pServoModule, "init_servos");
        pServoMove = PyObject_GetAttrString(pServoModule, "move_servos");
        
        if (!pNemaInit || !pNemaMove || !pServoInit || !pServoMove) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get Python functions");
            return;
        }
        
        // Initialize the motors
        PyObject* nema_args = PyTuple_Pack(1, PyUnicode_FromString("/dev/ttyUSB0"));
        PyObject* servo_args = PyTuple_Pack(1, PyUnicode_FromString("/dev/ttyUSB1"));
        
        if (PyObject_CallObject(pNemaInit, nema_args) == nullptr ||
            PyObject_CallObject(pServoInit, servo_args) == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize motors");
            return;
        }
        
        // Subscribe to joint states
        joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TTRobotController::joint_states_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "TTRobot controller initialized");
    }
    
    ~TTRobotController()
    {
        // Cleanup Python objects
        Py_DECREF(pNemaModule);
        Py_DECREF(pServoModule);
        Py_DECREF(pNemaInit);
        Py_DECREF(pNemaMove);
        Py_DECREF(pServoInit);
        Py_DECREF(pServoMove);
        
        // Finalize Python interpreter
        Py_Finalize();
    }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Find the indices of our joints in the message
        std::vector<int> joint_indices(5, -1);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "joint1") joint_indices[0] = i;
            else if (msg->name[i] == "joint2") joint_indices[1] = i;
            else if (msg->name[i] == "joint3") joint_indices[2] = i;
            else if (msg->name[i] == "joint4") joint_indices[3] = i;
            else if (msg->name[i] == "joint5") joint_indices[4] = i;
        }
        
        // Check if we found all joints
        for (int idx : joint_indices) {
            if (idx == -1) {
                RCLCPP_ERROR(this->get_logger(), "Could not find all joints in joint states message");
                return;
            }
        }
        
        // Convert joint positions to motor commands
        // Joint 1 (NEMA) - Convert from meters to steps
        double joint1_steps = msg->position[joint_indices[0]] * 2000.0; // Assuming 2000 steps/meter
        
        // Joint 2 (NEMA) - Convert from radians to degrees
        double joint2_degrees = msg->position[joint_indices[1]] * 180.0 / M_PI;
        
        // Joints 3-5 (ST3215) - Convert from radians to degrees
        double joint3_degrees = msg->position[joint_indices[2]] * 180.0 / M_PI;
        double joint4_degrees = msg->position[joint_indices[3]] * 180.0 / M_PI;
        double joint5_degrees = msg->position[joint_indices[4]] * 180.0 / M_PI;
        
        // Create Python arguments for NEMA motors
        PyObject* nema_args = PyTuple_Pack(2,
            PyFloat_FromDouble(joint1_steps),
            PyFloat_FromDouble(joint2_degrees));
            
        // Create Python arguments for servo motors
        PyObject* servo_args = PyTuple_Pack(3,
            PyFloat_FromDouble(joint3_degrees),
            PyFloat_FromDouble(joint4_degrees),
            PyFloat_FromDouble(joint5_degrees));
        
        // Move the motors
        if (PyObject_CallObject(pNemaMove, nema_args) == nullptr ||
            PyObject_CallObject(pServoMove, servo_args) == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move motors");
        }
        
        // Clean up Python objects
        Py_DECREF(nema_args);
        Py_DECREF(servo_args);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
    
    // Python objects
    PyObject* pNemaModule;
    PyObject* pServoModule;
    PyObject* pNemaInit;
    PyObject* pNemaMove;
    PyObject* pServoInit;
    PyObject* pServoMove;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TTRobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 