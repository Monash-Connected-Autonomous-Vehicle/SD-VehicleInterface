#ifndef SD_LOGGER
#define SD_LOGGER

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

void set_logger_level(const std::string& level, std::shared_ptr<rclcpp::Node> node)
{
    if(level == "debug")
    {
        node->get_logger().set_level(rclcpp::Logger::Level::Debug);
    }
    else if(level == "info")
    {
        node->get_logger().set_level(rclcpp::Logger::Level::Info);
    }
    else if(level == "warn")
    {
        node->get_logger().set_level(rclcpp::Logger::Level::Warn);
    }
    else if(level == "error")
    {
        node->get_logger().set_level(rclcpp::Logger::Level::Error);
    }
    else if(level == "fatal")
    {
        node->get_logger().set_level(rclcpp::Logger::Level::Fatal);
    }
}
 
#define DEBUG(node, wait_ms, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), wait_ms, message); \
        } \
    } while(0) \

#define DEBUG_COND(node, condition, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_DEBUG_STREAM_EXPRESSION(node->get_logger(), condition, message); \
        } \
    } while(0) \

#define INFO(node, wait_ms, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), wait_ms, message); \
        } \
    } while(0) \

#define INFO_COND(node, condition, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_INFO_STREAM_EXPRESSION(node->get_logger(), condition, message); \
        } \
    } while(0) \

#define WARN(node, wait_ms, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), wait_ms, message); \
        } \
    } while(0) \

#define WARN_COND(node, condition, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_WARN_STREAM_EXPRESSION(node->get_logger(), condition, message); \
        } \
    } while(0) \

#define WARN_COND_THROTTLE(node, wait_ms, condition, message) \
    do \
    { \
        if(_sd_enable_logging && (condition)) \
        { \
            RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), wait_ms, message); \
        } \
    } while(0) \
 
#define ERROR(node, wait_ms, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), wait_ms, message); \
        } \
    } while(0) \

#define ERROR_COND(node, condition, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_ERROR_STREAM_EXPRESSION(node->get_logger(), condition, message); \
        } \
    } while(0) \

#define FATAL(node, wait_ms, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_FATAL_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), wait_ms, message); \
        } \
    } while(0) \

#define FATAL_COND(node, condition, message) \
    do \
    { \
        if(_sd_enable_logging) \
        { \
            RCLCPP_FATAL_STREAM_EXPRESSION(node->get_logger(), condition, message); \
        } \
    } while(0) \

#endif
