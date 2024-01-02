#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_current.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "current_position.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// smh I added these for current control ----------------------------------------------------
#define ADDR_GOAL_CURRENT 102
// #define ADDR_PRESENT_CURRENT 126

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 1000000 // 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

// smh I added these for current control ----------------------------------------------------
uint16_t goal_current = 0;
uint16_t present_position = 0;


// Constructor for the ReadWriteNode class, which is a subclass of Node
ReadWriteNode::ReadWriteNode()
: Node("read_write_node") // Initialize the Node with the name "read_write_node"
{
  RCLCPP_INFO(this->get_logger(), "Run read write node"); // Log the start of the node

  // Declare and initialize a ROS2 parameter 'qos_depth' with a default value of 10
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0; // Variable to store the value of qos_depth
  this->get_parameter("qos_depth", qos_depth); // Retrieve the value of 'qos_depth' parameter


  // Define Quality of Service (QoS) settings for the ROS2 subscriber
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // Create a subscriber for the "set_current" topic
  set_current_subscriber_ =
    this->create_subscription<SetCurrent>(
    "set_current",                                  // Topic name
    QOS_RKL10V,                                     // QoS profile
    [this](const SetCurrent::SharedPtr msg) -> void // Callback function for the subscriber
    {
      uint8_t dxl_error = 0; //Variable to store the error from the DYNAMIXEL
      // Current Value of X series is 2*** byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint16_t goal_current = static_cast<uint16_t>(msg->current); // (unsigned int)msg->current;  // Convert int32 -> uint32

      // Write Goal Current (length : 2 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write2ByteTxRx( // write4ByteTxRx(
        portHandler,  
        static_cast<uint8_t>(msg->id),  // (uint8_t) msg->id, // Dynamixel ID
        ADDR_GOAL_CURRENT,              // ADDR_GOAL_POSITION, // Address to write the goal position
        goal_current,                   // Goal current value
        &dxl_error                      // Error storage
      );

      // Check and handle communication results or errors
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Current: %d]", msg->id, msg->current);
      }
    }
    );

  // Create a subscriber for the "set_position" topic
  set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

      // Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_POSITION,
        goal_position,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
      }
    }
    );

  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id, //static_cast<uint8_t>(request->id), 
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position), //&present_current, 
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

ReadWriteNode::~ReadWriteNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{

  // Use Current Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    0,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Current Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Current Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}


int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}