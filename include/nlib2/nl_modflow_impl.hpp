#ifndef NL_MODFLOW_IMPL_HPP
#define NL_MODFLOW_IMPL_HPP

#include <iostream>
#ifndef NL_MODFLOW_H
#include "nl_modflow.h"
#endif

// DEBUG ONLY
#include <iostream>
using std::cout;
using std::endl;

namespace nlib2 {

/***
 * class Event
 ***/

inline Event::SharedPtr Event::branch(const Module *module, const Channel *channel) {
    return Event::make_shared (_parent, module, channel);
}

inline bool Event::channelInAncestors(const std::string &name) const {
    const bool currentChannel = _channel->name () == name;
    
    if (_parent == nullptr)
        return currentChannel;
    
    return currentChannel || _parent->moduleInAncestors (name);
}

inline bool Event::moduleInAncestors(const std::string &name) const {
    const bool currentModule = _module->name() == name;
    
    if (_parent == nullptr)
        return currentModule;
    
    return currentModule || _parent->channelInAncestors (name);
}

inline const std::string &Event::moduleName () const {
    return _module->name ();
}

inline const std::string &Event::channelName () const {
    return _channel->name();
}

/***
 * class Channel
 ***/

template<typename ...typeids>
Channel::Channel (ChannelId id,
                 const std::string &name,
                 const Module *owner,
                 bool isSink,
                 const typeids *...ids):
    _id(id),
    _isSink(isSink),
    _name(name),
    _types(stackTypes (ids...)),
    _owner(owner)
{}

template<typename ...typeids>
std::vector<std::type_index> Channel::stackTypes (const typeids *...ids) {
    return {std::type_index(*ids) ...};
}

inline ChannelId Channel::id() const {
    return _id;
}

inline const std::string &Channel::name() const {
    return _name;
}

inline std::vector<std::type_index> Channel::types() const {
    return _types;
}

inline std::string Channel::ownerName() const {
    return _owner->name ();
}

inline bool Channel::checkOwnership(const Module *caller) const {
    if (_isSink)
        return true; // any module can emit on a sink
    return caller == _owner;
}

template<typename ...ChannelTs>
bool Channel::checkType () const {
    return stackTypes(&typeid(ChannelTs)...) == _types;
}

/***
 * class ResourceManager
 ***/

template<typename ResourceT, typename ...Args>
void ResourceManager::create(const std::string &name, Args &&...args) {
    _resources[name] = std::make_shared<ResourceT> (std::forward (args)...);
}

template<typename ResourceT>
std::shared_ptr<ResourceT> ResourceManager::get(const std::string &name) {
    auto resource = _resources[name];
    
    if (resource.type() != typeid (std::shared_ptr<ResourceT>))
        throw std::runtime_error ("Resource " + name + " has type " + resource.type().name() + ". Got " + typeid(std::shared_ptr<ResourceT>).name());
    
    return std::any_cast<std::shared_ptr<ResourceT>> (resource);
}

/***
 * class Module
 ***/


template<typename ...ChannelTs>
std::string errorChannelTypeMismatch (const Channel &channel, const Module *caller, bool emitting)
{
    std::stringstream ss;
    
    ss << "Type mismatch error on module " << caller->name () << "\n";
    ss << "Channel " << channel.name() << " requires {";
    for (auto currType : channel.types ()) {
        ss << abi::__cxa_demangle (currType.name (), NULL, NULL, NULL) << ", ";
    }
    ss << "}\n" << (emitting?  "while emitting types" : "connecting to slot with types")<< " {";
    std::vector<const char *> gotTypes{typeid(ChannelTs).name()...};
    for (const char *currTypeName : gotTypes) {
        ss << abi::__cxa_demangle (currTypeName, NULL, NULL, NULL) << ", ";
    }
    ss << "}\n";
    
    return ss.str();
}

inline std::string errorOwnership (const Channel &channel, const Module *caller) {
    return "Module " + caller->name () + " cannot emit on channel "
           + channel.name () + ", owned by " + channel.ownerName () + "\n";
}

template<typename ...ChannelTs, typename DerivedModule, typename ReturnT>
std::enable_if_t<std::is_base_of_v<Module,DerivedModule>>
Module::requestConnection (const std::string &channelName, ReturnT (DerivedModule::*slot)(ChannelTs ...))
{
    Channel channel = _modflow->resolveChannel (channelName);
    if (!channel.checkType<ChannelTs...> ()) {
        std::string msg = errorChannelTypeMismatch<ChannelTs...> (channel, this, false);
        throw std::runtime_error (msg);
    }
    
    Slot<ReturnT, ChannelTs...> boundSlot = [this, slot] (const Event::SharedPtr &event, ChannelTs ...args) -> ReturnT {
        DerivedModule *derived = dynamic_cast<DerivedModule *> (this);
        this->lastEvent() = event;
        if (this->isEnabled())
            return (derived->*slot) (args...);
    };
    
    _modflow->createConnection(channel, boundSlot, getFcnName (slot));
}

inline void Module::requestEnablingChannel (const Channel &channelId)
{
    Slot<void> boundEnableSlot = [this, channelId] (const Event::SharedPtr &event) {
        this->_lastEvent = event;
        this->setEnabled(channelId.id ());
    };
    
    _disablingChannels.insert(channelId.id());
    _modflow->createConnection(channelId, boundEnableSlot, "<enabling " + channelId.name()  + "> [" + name() + "]");
}

inline void Module::requestEnablingChannel (const std::string &channelName) {
    Channel channel = _modflow->resolveChannel(channelName);
    requestEnablingChannel (channel);
}

inline void Module::setEnabled (ChannelId enablingChannelId) {
    // if already erased simply ignore
    _disablingChannels.erase(enablingChannelId);
}

inline bool Module::isEnabled () const {
    return _disablingChannels.empty();
}

inline void Module::onParamChange (const std::string &name, const NlParams::SharedPtr &nlParams) {
    if (_automaticParamUpdate)
        initParams (nlParams);
    else
        updateParam (name, nlParams);
}

template<typename ...ChannelTs>
Channel Module::createChannel (const std::string &name) {
    return _modflow->createChannel<ChannelTs...> (name, this);
}

template<typename ...ChannelTs>
Channel Module::requireSink (const std::string &sinkName)
{
    Channel sink = _modflow->resolveChannel(sinkName);
    
    if (!sink.checkType<ChannelTs...> ())
        throw std::runtime_error (errorChannelTypeMismatch<ChannelTs...> (sink, this, false));
    
    return sink;
}

template<typename ...ChannelTs>
void Module::emit (const Channel &channel, ChannelTs &&...args) {
    _modflow->emit<void, ChannelTs...> (channel, this, std::forward<ChannelTs> (args) ...);
}

template<typename ...ChannelTs>
void Module::emit (const std::string &channelName, ChannelTs &&...args) {
    _modflow->emit<void, ChannelTs...> (channelName, this, std::forward<ChannelTs> (args) ...);
}

template<typename ReturnT, typename ...ChannelTs>
ReturnT Module::callService (const Channel &channel, ChannelTs &&...args) {
    return _modflow->emit<ReturnT, ChannelTs...> (channel, this, std::forward<ChannelTs> (args)...);
}

template<typename ReturnT, typename ...ChannelTs>
ReturnT Module::callService (const std::string &channelName, ChannelTs &&...args) {
    return _modflow->emit<ReturnT, ChannelTs...> (channelName, this, std::forward<ChannelTs> (args) ...);
}

inline const ResourceManager &Module::resources () const  { return _modflow->_resources; }
inline ResourceManager &Module::resources ()  { return _modflow->_resources; }

/***
 * class Sources
 ***/

template<typename ...ChannelTs>
Channel Sources::declareSourceChannel (const std::string &name) {
    return _modflow->createChannel<ChannelTs...> (name, this);
}

template<typename ...ChannelTs>
void Sources::callSource (const Channel &channel, ChannelTs &&...args) {
    emit<ChannelTs...> (channel, std::forward<ChannelTs> (args) ...);
}

template<typename ...ChannelTs>
void Sources::callSource (const std::string &channel, ChannelTs &&...args) {
    emit<ChannelTs...> (channel, std::forward<ChannelTs> (args) ...);
}

/***
 * class Sinks
 ***/

template<typename ...ChannelTs, typename Callback>
void Sinks::declareSink (const std::string &name, const Callback &parentCallback)
{
    Channel channel = _modflow->createChannel<ChannelTs...> (name, this, true);
    
    Slot<void, ChannelTs...> boundSlot = [parentCallback] (const Event::SharedPtr &, ChannelTs &&...args) {
        parentCallback (std::forward<ChannelTs> (args) ...);
    };
    
    _modflow->createConnection(channel, boundSlot, TYPE(Callback));
}

/***
 * class SerializedSlot
 ***/

template<typename ReturnT, typename ...SlotTypes>
SerializedSlot::SerializedSlot(const Slot<ReturnT, SlotTypes...> &slt,
                               const Channel &channel,
                               const std::string &slotName,
                               std::enable_if_t<(sizeof ...(SlotTypes) <= 1)> *):
    _name(slotName),
    _channel(channel)
{
    serialize (slt);
}

template<typename ReturnT, typename ...SlotTypes>
SerializedSlot::SerializedSlot(const Slot<ReturnT, SlotTypes...> &slt,
                               const Channel &channel,
                               const std::string &slotName,
                               std::enable_if_t<(sizeof ...(SlotTypes) > 1)> *):
    _name(slotName),
    _channel(channel)
{
    serialize (slt, std::make_index_sequence<sizeof ...(SlotTypes)> ());
}


template<typename ...SlotTypes>
std::enable_if_t<(sizeof ...(SlotTypes) <= 1)>
SerializedSlot::serialize (const Slot<void, SlotTypes...> &slot)
{
    _serialized = [slot] (const Event::SharedPtr &event, const std::any &args) -> std::any {
        slot(event, std::any_cast<SlotTypes> (args) ...); // expands to either slot() or slot(args), if 0 or 1 arguments
        
        return std::any ();
    };
}

template<typename ...SlotTypes, std::size_t ...is>
std::enable_if_t<(sizeof ...(SlotTypes) > 1)>
SerializedSlot::serialize (const Slot<void, SlotTypes...> &slot,
                          std::index_sequence<is...>)
{
    _serialized = [slot] (const Event::SharedPtr &event, const std::any &args) -> std::any {
        auto argsTuple = std::any_cast<std::tuple<SlotTypes...>> (args);
        slot (event, std::get<is> (argsTuple) ...);
        
        return std::any ();
    };
}

template<typename ReturnT, typename ...SlotTypes>
std::enable_if_t<(sizeof ...(SlotTypes) <= 1) &&
                 !std::is_same_v<ReturnT, void>>
SerializedSlot::serialize (const Slot<ReturnT, SlotTypes...> &slot)
{
    _serialized = [slot] (const Event::SharedPtr &event, const std::any &args) -> std::any {
        return slot(event, std::any_cast<SlotTypes> (args) ...);
    };
}

template<typename ReturnT, typename ...SlotTypes, std::size_t ...is>
std::enable_if_t<(sizeof ...(SlotTypes) > 1) &&
                 !std::is_same_v<ReturnT, void>>
SerializedSlot::serialize (const Slot<ReturnT, SlotTypes...> &slot,
                          std::index_sequence<is...>)
{
    _serialized = [slot] (const Event::SharedPtr &event, const std::any &args) -> std::any {
        auto argsTuple = std::any_cast<std::tuple<SlotTypes...>> (args);
        return slot (event, std::get<is> (argsTuple) ...);
    };
}

template<typename ReturnT, typename SlotT>
std::enable_if_t<std::is_same_v<ReturnT, void>, ReturnT>
SerializedSlot::invoke (const Event::SharedPtr &event, SlotT &&arg) const {
    _serialized (event, std::forward<SlotT> (arg));
}

template<typename ReturnT, typename ...SlotTypes>
std::enable_if_t<(sizeof ...(SlotTypes) > 1) &&
                     std::is_same_v<ReturnT, void>, ReturnT>
SerializedSlot::invoke (const Event::SharedPtr &event, SlotTypes &&...args) const {
    _serialized (event, std::make_tuple(std::forward<SlotTypes> (args) ...));
}


template<typename ReturnT>
std::enable_if_t<std::is_same_v<ReturnT, void>, ReturnT>
SerializedSlot::invoke (const Event::SharedPtr &event) const {
    _serialized (event, std::any ());
}

template<typename ReturnT, typename SlotT>
std::enable_if_t<!std::is_same_v<ReturnT, void>, ReturnT>
SerializedSlot::invoke (const Event::SharedPtr &event, SlotT &&arg) const {
    return std::any_cast<ReturnT> (_serialized (event, std::forward<SlotT> (arg)));
}

template<typename ReturnT, typename ...SlotTypes>
std::enable_if_t<(sizeof ...(SlotTypes) > 1) &&
                     !std::is_same_v<ReturnT, void>, ReturnT>
SerializedSlot::invoke (const Event::SharedPtr &event, SlotTypes &&...args) const
{
    return std::any_cast<ReturnT> (
        _serialized (
            event,
            std::make_tuple(
                std::forward<SlotTypes> (args) ...)));
}

template<typename ReturnT>
std::enable_if_t<!std::is_same_v<ReturnT, void>, ReturnT>
SerializedSlot::invoke (const Event::SharedPtr &event) const {
    return std::any_cast<ReturnT> (_serialized (event, std::any ()));
}

/***
 * class Modflow
 ***/

inline Modflow::Modflow ():
    _channelsSeq(0)
{}

template<class DerivedModflow>
std::enable_if_t<std::is_base_of_v<Modflow, DerivedModflow>,
                 Modflow::SharedPtr>
Modflow::create () {
    return std::make_shared<DerivedModflow> ();
}

inline void Modflow::init (const NlParams::SharedPtr &nlParams)
{
    _nlParams = nlParams;
    
    initDebugConfiguration ();
    
    _sources = loadModule<Sources> ();
    _sinks = loadModule<Sinks> ();
    
    loadModules ();
}

inline void Modflow::finalize ()
{
    for (const Module::SharedPtr &module : _modules) {
        if (hasParams ())
            module->initParams(_nlParams->derive(module->name()));
        module->setupNetwork();
    }
}

inline Sources::SharedPtr Modflow::sources () {
    return _sources;
}

inline Sinks::SharedPtr Modflow::sinks () {
    return _sinks;
}

inline std::pair<std::string, std::string> splitParamPath (const std::string &fullName) {
    size_t separatorPos = fullName.find('.');

    if (separatorPos == std::string::npos)
        throw std::runtime_error ("Bad parameter name " + fullName);

    return {fullName.substr(0, separatorPos),
            fullName.substr(separatorPos + 1)};
}

inline void Modflow::onParameterChange (const std::string &fullName, const NlParams::SharedPtr &nlParams)
{
    auto [moduleName, paramName] = splitParamPath (fullName);

    for (const Module::SharedPtr &currModule : _modules)
        if (currModule->name() == moduleName) {
            currModule->onParameterChange(paramName, nlParams->derive(moduleName));
            break;
        }
}

inline std::string errorModuleNameConflict (const std::string &name, const Module *owner)
{
    std::stringstream ss;
    
    ss << "Module " << owner->name () << " creating channel "
       << name << ": already exists" << std::endl;
    
    return ss.str();
}

inline std::string errorUnresolvedChannel (const std::string &name) {
    return "Channel " + name + " does not exist\n";
}

inline std::string errorMultipleConnections () {
    return "Non-void return type only allowed to channels with single connections";
}


inline std::string truncateArguments (const std::string &fcn) {
    return fcn.substr (0, fcn.find ('('));
}

inline void debugConnection (int depth, const  Module *caller, const SerializedSlot &slot) {
    std::cout << "\e[33m[ModFlow] [" << printTime(std::chrono::system_clock::now ()) << "]\e[0m "
              << std::string (depth, '+') << (depth > 0 ? " " : "")
              << "\e[32m" << caller->name () << "\e[0m"
              << " calling slot \e[36m" << truncateArguments (slot.name ()) << "\e[0m"
              << (caller->isEnabled () ? "" : "(not enabled)")
              << std::endl;
}

template<typename ...ChannelTs>
Channel Modflow::createChannel (const std::string &name, const Module *owner, bool isSink)
{
    Channel newChannel(_channelsSeq, name, owner, isSink, &typeid(ChannelTs) ...);
    
    if (_channelNames.find (name) != _channelNames.end())
        throw std::runtime_error(errorModuleNameConflict (name, owner));
    
    _channelNames[name] = newChannel;
    _connections.push_back({});
    _channelsSeq++;
    
    return newChannel;
}


inline Channel Modflow::resolveChannel (const std::string &name)
{
    auto channelIt = _channelNames.find (name);
    
    if (channelIt == _channelNames.end ())
        throw std::runtime_error (errorUnresolvedChannel (name));
    
    return channelIt->second;
}

template<typename ReturnT, typename ...ChannelTs>
void Modflow::createConnection (const Channel &channel,
                               const Slot<ReturnT, ChannelTs...> &slot,
                               const std::string &name)
{
    SerializedSlot serializedSlot(slot, channel, name);
    
    _connections[channel.id()].push_back(serializedSlot);
}

template<typename ReturnT, typename ...ChannelTs>
std::enable_if_t<!std::is_same_v<ReturnT, void>, ReturnT>
Modflow::emit (const Channel &channel, const Module *caller, ChannelTs &&...args)
{
    Event::SharedPtr event = prepareEmit<ReturnT, ChannelTs...> (channel, caller);
    
    if (_connections[channel.id()].size() != 1)
        throw std::runtime_error (errorMultipleConnections());
    
    const SerializedSlot &currentSlot = _connections[channel.id()].front();
    
    if (debugFilters (event))
        debugConnection (event->depth(), caller, currentSlot);
    
    return currentSlot.invoke<ReturnT, ChannelTs...> (event, std::forward<ChannelTs> (args)...);
}

template<typename ReturnT, typename ...ChannelTs>
std::enable_if_t<std::is_same_v<ReturnT, void>, ReturnT>
Modflow::emit (const Channel &channel, const Module *caller, ChannelTs &&...args)
{
    Event::SharedPtr event = prepareEmit<ReturnT, ChannelTs...> (channel, caller);
    
    for (const SerializedSlot &currentSlot : _connections[channel.id()]) {
        if (debugFilters (event))
            debugConnection (event->depth(), caller, currentSlot);
        
        currentSlot.invoke<ReturnT, ChannelTs...> (event, std::forward<ChannelTs> (args) ...);
    }
}

template<typename ReturnT, typename ...ChannelTs>
ReturnT Modflow::emit (const std::string &channelName, const Module *caller, ChannelTs &&...args)
{
    return emit<ReturnT, ChannelTs...> (resolveChannel (channelName), caller, std::forward<ChannelTs> (args) ...);
}

template<class DerivedModule, typename ...Args>
typename DerivedModule::SharedPtr Modflow::loadModule (Args &&...args) {
    auto newModule = std::make_shared<DerivedModule> (shared_from_this(), std::forward<Args> (args)...);
    
    _modules.push_back(std::dynamic_pointer_cast<Module> (newModule));
    
    return newModule;
}

inline void debugTrackEmit (int depth, const Channel &channel, const Module *caller, int connectionsCount)
{
    std::string connectionsStr = "(" + std::to_string (connectionsCount) + " connection" + (connectionsCount > 1 ? "s" : "") + ")";
    std::cout << (depth == 0 ? "\n" : "")
              << "\e[33m[ModFlow] [" << printTime(std::chrono::system_clock::now ()) << "]\e[0m "
              << std::string (depth, '+' ) << (depth > 0 ? " " : "")
              << "Module \e[32m" << caller->name ()
              << "\e[0m emitted \e[93m" << channel.name () << "\e[0m "
              << (connectionsCount > 0? connectionsStr : "(no connections)")
              << std::endl;
}

template<typename ReturnT, typename ...ChannelTs>
Event::SharedPtr Modflow::prepareEmit (const Channel &channel, const Module *caller)
{
    if (!channel.checkType<ChannelTs...> ())
        throw std::runtime_error (errorChannelTypeMismatch<ChannelTs...> (channel, caller, true));
    
    if (!channel.checkOwnership(caller))
        throw std::runtime_error (errorOwnership (channel, caller));
    
    Event::SharedPtr lastEvent = caller->lastEvent();
    Event::SharedPtr event;
    
    if (lastEvent == nullptr)
        event = std::make_shared<Event> (caller, &channel);
    else
        event = std::make_shared<Event> (lastEvent, caller, &channel);
    
    if (debugFilters (event))
        debugTrackEmit (event->depth(), channel, caller, _connections[channel.id()].size ());
    
    return event;
}

inline void Modflow::initDebugConfiguration ()
{
    if (!hasParams ()) {
        _debug.enabled = false;
        return;
    }
    
    _debug.enabled = _nlParams->declareAndGet<bool> ("modflow.debug.enable");
    
    if (!_debug.enabled)
        return;
    
    _debug.filterOnlyChannels = _nlParams->declareAndGet<std::vector<std::string>> ("modflow.debug.only_channels", std::vector<std::string> ());
    _debug.filterOnlyModules = _nlParams->declareAndGet<std::vector<std::string>> ("modflow.debug.only_modules", std::vector<std::string> ());
    
    _debug.filterExcludeChannels = _nlParams->declareAndGet<std::vector<std::string>> ("modflow.debug.exclude_channels", std::vector<std::string> ());
    _debug.filterExcludeModules = _nlParams->declareAndGet<std::vector<std::string>> ("modflow.debug.exclude_modules", std::vector<std::string> ());
}

inline bool Modflow::debugFilters (const Event::SharedPtr &event)
{
    if (!_debug.enabled)
        return false;
    
    for (const std::string &curr : _debug.filterOnlyChannels) {
        if (!event->channelInAncestors (curr))
            return false;
    }
    
    for (const std::string &curr : _debug.filterOnlyModules) {
        if (!event->moduleInAncestors (curr))
            return false;
    }
    
    for (const std::string &curr : _debug.filterExcludeChannels) {
        if (event->channelInAncestors (curr))
            return false;
    }
    
    for (const std::string &curr : _debug.filterExcludeModules) {
        if (event->moduleInAncestors (curr))
            return false;
    }
    
    return true;
}


}

#endif // NL_MODFLOW_IMPL_HPP
