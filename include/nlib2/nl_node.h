#ifndef NL_NODE_H
#define NL_NODE_H

#include <type_traits>
#include "rclcpp/rclcpp.hpp"
#include "nl_utils.h"
#include "nl_params.h"
#include <string>

namespace nlib2 {

class ROS2ParameterServer : public IParameterServer
{
    enum class Call {
        DECLARE,
        GET
    };

public:
    NLIB_DEF_SMARTPTR(ROS2ParameterServer)

    ROS2ParameterServer (const rclcpp::Node::SharedPtr &node): _node(node) { }

protected:
    void declareParameter (const std::string &name, const std::optional<std::any> &defaultValuie, const std::type_index &type);
    std::any getParameter (const std::string &name, const std::type_index &type);

private:
    template<typename ParameterT>
    std::any processResolved (Call callType, const std::string &name, const std::optional<std::any> &defaultValue);
    std::any resolve (const std::type_index &type, Call callType, const std::string &name, const std::optional<std::any> &defaultValue);

    template<typename currType, typename ...otherTypes>
    std::any resolveOne (const std::type_index &type, Call callType, const std::string &name, const std::optional<std::any> &defaultValue);

private:
    rclcpp::Node::SharedPtr _node;
};

class RosConfiguration
{
public:
    NLIB_DEF_SMARTPTR(RosConfiguration)

    RosConfiguration (const rclcpp::Node::SharedPtr &node):
        _node(node)
    {}

    void setup (bool centralized, const std::string &centralizedNodeName = "");
    std::string getTopic (const std::string &name, bool sub);

private:
    bool waitAlive ();

private:
    bool _centralized;
    std::string _centralizedNodeName;
    rclcpp::Node::SharedPtr _node;
    rclcpp::SyncParametersClient::SharedPtr _parameterClient;
};

template<typename Derived>
class NlNode
{
public:
    NLIB_DEF_SMARTPTR(NlNode)

    NlNode (int argc, char **argv, const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions ());

    int spin ();
    const std::string &name () const { return _name; }

protected:
    NLIB_CRTP_DERIVED(Derived)

    void initParams ();
    void initROS ();

    template<typename MessageT>
    void addSubscription (const std::string &name,
                         const std::string &topic,
                         const rclcpp::QoS &qos,
                         void (Derived::*callback) (MessageT));

    template<typename MessageT>
    void addSubscription (const std::string &name,
                         const rclcpp::QoS &qos,
                         void (Derived::*callback) (MessageT));

    template<typename MessageT>
    void addPublisher (const std::string &name,
                      const std::string &topic,
                      const rclcpp::QoS &qos);

    template<typename MessageT>
    void addPublisher (const std::string &name,
                      const rclcpp::QoS &qos);

    template<typename MessageT>
    void publish (const std::string &name, const MessageT &msg);

    void onParamChange (const std::string &paramName) { }
    void onClock () { }

private:
    void initParamUpdates ();
    void initClock ();
    void resetClock (float clockTime_ms);
    void setClock (float clockTime_ms);

protected:
    std::string _name;
    rclcpp::Node::SharedPtr _node;
    ROS2ParameterServer::SharedPtr _parameterServer;
    NlParams::SharedPtr _nlParams;
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> _publishers;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> _subscriptions;
    int _argc;
    char **_argv;

private:
    RosConfiguration::SharedPtr _rosConfiguration;
    std::shared_ptr<rclcpp::ParameterEventHandler> _parameterEventHandler;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> _parameterCbHandle;
    rclcpp::TimerBase::SharedPtr _clockHandle;
};

}

// Template implementations
#ifndef NL_NODE_IMPL_HPP
#include "nl_node_impl.hpp"
#endif
#endif // NL_NODE_H
