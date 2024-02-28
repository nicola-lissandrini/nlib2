#ifndef NL_NODE_IMPL_HPP
#define NL_NODE_IMPL_HPP

#ifndef NL_NODE_H
#include "nl_node.h"
#endif

#include <std_srvs/srv/empty.hpp>

namespace nlib2 {

namespace internal::traits {

template<typename T, typename = void>
struct is_container : std::false_type {};

template<typename T>
struct is_container<T, std::void_t<decltype(std::declval<T>().begin()), decltype(std::declval<T>().end())>> : std::true_type {};

}

template<typename ParameterT>
std::any ROS2ParameterServer::processResolved (Call callType, const std::string &name, const std::optional<std::any> &defaultValue)
{
    switch (callType) {
    case Call::DECLARE:
        if (defaultValue.has_value())
            return _node->declare_parameter<ParameterT> (name, std::any_cast<ParameterT> (*defaultValue));
        else
            return _node->declare_parameter<ParameterT> (name);
        break;
    case Call::GET:
        auto param = _node->get_parameter (name).get_value<ParameterT> ();
        // ROS2 stores parameters in 64bits, so if you ask e.g. a float instead of a double, it will return a double. So if you ask for a float, it needs to be converted
        if constexpr (internal::traits::is_container<ParameterT>::value) // convert vector to correct type (e.g. float instead of double)
            return ParameterT (param.begin (), param.end ());
        else
            return static_cast<ParameterT> (param);
    }

    return std::any ();
}

void ROS2ParameterServer::declareParameter (const std::string &name, const std::optional<std::any> &defaultValue, const std::type_index &type) {
    resolve (type, Call::DECLARE, name, defaultValue);
}

std::any ROS2ParameterServer::getParameter (const std::string &name, const std::type_index &type) {
    return resolve (type, Call::GET, name, std::nullopt);
}

template<typename currType, typename ...otherTypes>
std::any ROS2ParameterServer::resolveOne(const std::type_index &type, Call callType, const std::string &name, const std::optional<std::any> &defaultValue) {
    if (type == typeid(currType))
        return processResolved<currType> (callType, name, defaultValue);
    else {
        if constexpr (sizeof... (otherTypes) == 0)
            throw std::runtime_error("Unsupported type");
        else
            return resolveOne<otherTypes...> (type, callType, name, defaultValue);
    }
}

std::any ROS2ParameterServer::resolve (const std::type_index &type, Call callType, const std::string &name, const std::optional<std::any> &defaultValue)
{
    return resolveOne<
        bool, int, float, double,
        std::string,
        std::vector<uint8_t>,
        std::vector<bool>,
        std::vector<int>,
        std::vector<int64_t>,
        std::vector<float>,
        std::vector<double>,
       std::vector<std::string>> (type, callType, name, defaultValue);
}

template<typename Derived>
NlNode<Derived>::NlNode(int argc, char **argv, const std::string &name, const rclcpp::NodeOptions &options):
    _argc(argc),
    _argv(argv)
{
    static_assert (std::is_base_of_v<NlNode, Derived>, "Derived must inherit from NlNode");

    rclcpp::init(_argc, _argv);
    _node = std::make_shared<rclcpp::Node> (name, options);
    initParams ();
    initROS ();
}

void RosConfiguration::setup (bool centralized, const std::string &centralizedNodeName)
{
    _centralized = centralized;
    _centralizedNodeName = centralizedNodeName;

    if (!_centralized) {
        _parameterClient = nullptr;
        return;
    }

    if (!waitAlive ())
        throw std::runtime_error (centralizedNodeName + " does not appear to be alive");

    _parameterClient = std::make_shared<rclcpp::SyncParametersClient> (_node, _centralizedNodeName);
}

bool RosConfiguration::waitAlive ()
{
    using namespace std::chrono_literals;

    constexpr auto TIMEOUT = 5s;

    auto client = _node->create_client<std_srvs::srv::Empty>("/ros_configuration_alive");
    auto request = std::make_shared<std_srvs::srv::Empty::Request> ();

    if (!client->wait_for_service(TIMEOUT))
        return false;

    auto result = client->async_send_request(request);

    return rclcpp::spin_until_future_complete(_node, result, TIMEOUT) == rclcpp::FutureReturnCode::SUCCESS;
}

std::string RosConfiguration::getTopic (const std::string &name, bool sub)
{
    using namespace std::string_literals;

    std::string topicPath = _node->get_name() + ".topics."s + (sub ? "subs."s : "pubs."s) + name;

    if (_centralized)
        return _parameterClient->get_parameter<std::string> (topicPath);
    else {
        _node->declare_parameter<std::string> (topicPath);
        return _node->get_parameter (topicPath).get_value<std::string> ();
    }
}

template<typename Derived>
void NlNode<Derived>::initParams()
{
    _parameterServer = ROS2ParameterServer::make_shared(_node);
    _nlParams = NlParams::initialize(_parameterServer);
    _rosConfiguration = std::make_shared<RosConfiguration> (_node);

    bool rosConfigurationEnable = _nlParams->declareAndGet<bool> ("ros_configuration_enable", true);

    if (rosConfigurationEnable)
        _rosConfiguration->setup(true,
                                 _nlParams->declareAndGet<std::string> ("ros_configuration_node","/ros_configuration"));
    else
        _rosConfiguration->setup(false);
}

template<typename Derived>
void NlNode<Derived>::initROS ()
{
    initParamUpdates ();
    initClock ();
}

inline constexpr auto CLOCK_PARAM= "clock_time_ms";

template<typename Derived>
void NlNode<Derived>::initParamUpdates ()
{
    _parameterEventHandler = std::make_shared<rclcpp::ParameterEventHandler> (_node);

    auto parameterCallback = [this] (const rcl_interfaces::msg::ParameterEvent &event) {
        for (const auto &changedParam : event.changed_parameters) {
            if (changedParam.name == CLOCK_PARAM) {
                this->resetClock (changedParam.value.double_value);
                continue;
            }

            this->derived().onParamChange (changedParam.name);
        }
    };

    _parameterCbHandle = _parameterEventHandler->add_parameter_event_callback (parameterCallback);
}

template<typename Derived>
void NlNode<Derived>::setClock (float clockTime_ms)
{
    auto clockCallback = [this] () {
        derived().onClock ();
    };

    _clockHandle = _node->create_wall_timer(std::chrono::duration<float, std::milli> (clockTime_ms), clockCallback);
}

template<typename Derived>
void NlNode<Derived>::resetClock (float clockTime_ms)
{
    _clockHandle->cancel();
    setClock (clockTime_ms);
}

template<typename Derived>
void NlNode<Derived>::initClock ()
{
    try {
        float clockTime_ms = _nlParams->declareAndGet<float> (CLOCK_PARAM);

        setClock (clockTime_ms);

    } catch (const rclcpp::exceptions::ParameterUninitializedException &)
    {
        // if the parameter is not set, do no set the clock
    }
}

template<typename Derived>
int NlNode<Derived>::spin ()
{
    rclcpp::executors::SingleThreadedExecutor executor;

    derived().initParams ();
    derived().initROS ();

    executor.add_node(_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}


template<typename Derived>
template<typename MessageT>
void NlNode<Derived>::addSubscription (const std::string &name, const std::string &topic, const rclcpp::QoS &qos, void (Derived::*callback)(MessageT))
{
    using MessageT_raw = std::remove_cv_t<std::remove_reference_t<MessageT>>;
    auto subscription = _node->create_subscription<MessageT_raw> (
        topic, qos,
        [this, callback] (MessageT msg) {
            (derived().*callback) (msg);
        } );
    _subscriptions.insert ({name, subscription});
}

template<typename Derived>
template<typename MessageT>
void NlNode<Derived>::addSubscription (const std::string &name,
                                      const rclcpp::QoS &qos,
                                      void (Derived::*callback)(MessageT))
{
    addSubscription (name, _rosConfiguration->getTopic (name, true), qos, callback);
}

template<typename Derived>
template<typename MessageT>
void NlNode<Derived>::addPublisher (const std::string &name, const std::string &topic, const rclcpp::QoS &qos)
{
    using MessageT_raw = std::remove_cv_t<std::remove_reference_t<MessageT>>;
    auto publisher = _node->create_publisher<MessageT_raw> (topic, qos);
    _publishers.insert({name, publisher});
}

template<typename Derived>
template<typename MessageT>
void NlNode<Derived>::addPublisher (const std::string &name, const rclcpp::QoS &qos)
{
    addPublisher<MessageT> (name, _rosConfiguration->getTopic (name, false), qos);
}


template<typename Derived>
template<typename MessageT>
void NlNode<Derived>::publish (const std::string &name,
                              const MessageT &msg) {
    return std::dynamic_pointer_cast<rclcpp::Publisher<MessageT>> (_publishers.at(name))->publish (msg);
}

}
#endif // NL_NODE_IMPL_HPP
