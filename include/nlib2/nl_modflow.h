#ifndef NL_MODFLOW_H
#define NL_MODFLOW_H

#include "nl_utils.h"
#include "nl_params.h"
#include <functional>
#include <set>
#include <typeindex>
#include <unordered_map>

/**
 * @file nl_modflow.h
 * @author Nicola Lissandrini
 */

namespace nlib2 {

using ChannelId = int64_t;

class Modflow;
class Module;
class Sources;
class Sinks;
class Channel;

class Event
{
public:
    NLIB_DEF_SMARTPTR(Event)

    Event (const Module *module,
          const Channel *channel):
        _parent(nullptr),
        _module(module),
        _channel(channel),
        _depth(0)
    {}

    Event (const Event::SharedPtr &parent,
          const Module *module,
          const Channel *channel):
        _parent(parent),
        _module(module),
        _channel(channel),
        _depth(parent->depth () + 1)
    {}

    Event::SharedPtr branch (const Module *module, const Channel *channel);
    bool channelInAncestors (const std::string &name) const;
    bool moduleInAncestors (const std::string &name) const;
    const std::string &moduleName () const;
    const std::string &channelName () const;

    int depth () const { return _depth; }


private:
    std::unordered_map<std::string, void * const> _resources;
    const Event::SharedPtr _parent;
    const Module *_module;
    const Channel *_channel;
    const int _depth;
};

template<typename R, typename ...T>
using Slot = std::function<R(const Event::SharedPtr &, T ...)>;

/**
 * @defgroup modflow Modflow: a graph based modular interface
 * @details
 */

/**
 * @brief Defines a channel that each module can create and to which other modules can connect
 * @ingroup modflow
 */

class Channel
{
    template<typename ...typeids>
    static std::vector<std::type_index> stackTypes (const typeids *...ids);

public:
    NLIB_DEF_SMARTPTR(Channel)

    /**
     * @brief Create a new channel given:
     * @param id  Unique identifier of the channel
     * @param name Unique name that can resolve to the id from a ModFlow handler
     * @param ids Channel type(s) identifier: only slots with same type(s) as channel can be connected
     * @param owner Pointer to owner module: only owner can emit events on channels has itself created
     * @param isSink Sink channels are connected to Parent methods, external to modflow
     */
    template<typename ...typeids>
    Channel (ChannelId id,
            const std::string &name,
            const Module *owner,
            bool isSink,
            const typeids *...ids);

    Channel (const Channel &) = default;
    Channel () = default;

    /// @brief Get unique identifier of the channel
    ChannelId id () const;

    /// @brief Get name of the channel
    const std::string &name () const;

    /// @brief Check whether supplied type is compatible with Channel type
    /// @tparam T Type(s) to check Channel-type with
    template<typename ...T>
    bool checkType () const;

    /// @brief Return vector of the types of the channel
    std::vector<std::type_index> types () const;

    /// @brief Return name of owner module
    std::string ownerName () const;

    /// @brief Verify that a module is the effective owner of the channel
    bool checkOwnership (const Module *caller) const;

private:
    ChannelId _id;
    bool _isSink;
    std::string _name;
    std::vector<std::type_index> _types;
    const Module *_owner;
};

/**
 * @brief Allow sharing resources of generic types among modules
 * @ingroup modflow
 */

class ResourceManager
{
public:
    /**
     * @brief Create a new resource on heap, stored with type erasure
     * @param name Unique name to access resource
     * @param args Resource constructor arguments
     */
    template<typename T, typename ...Args>
    void create (const std::string &name, Args &&...args);

    /**
     * @brief Get an existing resource of type T
     * @param name Resource unique name
     */
    template<typename T>
    std::shared_ptr<T> get (const std::string &name);

private:
    std::unordered_map<std::string, std::any> _resources;
};


/**
 * @brief This is the core node of a Modflow graph. Inherit this class to define the main computation units to happen in this module. Each module
 * can define new Channels to which it can emit output events after computation is done, and request channels to connect to when data is transmitted on
 * such channels
 * @ingroup modflow
 */

class Module
{
public:
    NLIB_DEF_SMARTPTR(Module)

protected:
    /**
     * @brief The module can only be created via Modflow::loadModule, which handles its allocation as shared pointer. For each
     * module, the constructor should have a Modflow shared pointer as first argument, and then the optional other argumetns, and forward
     * the pointer to the parent's constructor
     */
    Module (const std::shared_ptr<Modflow> &modflow,
           const std::string &name,
           bool automaticParamUpdate = true):
        _modflow(modflow),
        _lastEvent(nullptr),
        _name(name),
        _automaticParamUpdate(automaticParamUpdate)
    {}

public:
    /// @brief Get module name
    const std::string &name () const { return _name; }

    /// @brief For internal use
    Event::SharedPtr lastEvent () const { return _lastEvent; }

    /**
     * @brief Override this method to (optionally) initialize the parameters
     * Parameters are supplied during the execution of @ref Modflow::finalize.
     * The nlParams object is already scoped with the parameters accessed
     * by e.g. "moduleX" must be defined as moduleX.params
     * @param nlParams Obtained as nlParams->derived(module->name())
     */
    virtual void initParams (const NlParams::SharedPtr &nlParams) {}

    /**
     * @brief Implement this method to handle the connections.
     * Create channels (see @ref Module::createChannel) for outbound connections
     * and request connections from inbound channels (see @ref Module::requestConnection)
     */
    virtual void setupNetwork () = 0;

    /**
     * @brief Retruns true when all "enabling channels" have been triggered at least once
     * (see @ref requestEnablingChannel)
     */
    bool isEnabled () const;

    /**
     * @brief Should only be called by the Modflow object. If _automaticParamUpdate is true,
     * it re-execute initParams, and every parameter of the module is re-read.
     * Otherwise, it calls your implementation of Module::updateParam
     * @param name Scoped parameter name
     * @param nlParams NlParam object pointer
     */
    void onParamChange (const std::string &name, const NlParams::SharedPtr &nlParams);

protected:
    /**
     * @brief This function is called after onParamChange when _automaticParamUpdate is false
     * @param name
     * @param nlParams
     */
    virtual void updateParam (const std::string &name, const NlParams::SharedPtr &nlParams) {}

    /**
     * @brief Bind signals emitted on a given channel name to a member function @p slot of the derived module
     * The channel types are automatically deduced from the slot arguments
     * @see Modflow::createConnection
     */
    template<typename ...ChannelTs, typename DerivedModule, typename ReturnT>
    std::enable_if_t<std::is_base_of_v<Module,DerivedModule>>
    requestConnection (const std::string &channelName, ReturnT (DerivedModule::*slot)(ChannelTs ...));

    /**
     * @brief Request a channel to be enabling of the module.
     * Until all enabling channels have been triggered at least once,
     * all other inbound connections are disabled
     */
    void requestEnablingChannel (const std::string &channelName);
    void requestEnablingChannel (const Channel &channelName);

    /**
     * @brief Create a standard channel of types @p ChannelTs named @p name, owned by this module
     * (@see Modflow::createChannel for details)
     */
    template<typename ...ChannelTs>
    Channel createChannel (const std::string &name);

    /**
     * @brief Ensures the parent object has declared a sink named @p sinkName with types @p ChannelTs
     */
    template<typename ...ChannelTs>
    Channel requireSink (const std::string &sinkName);

    /**
     * @brief Emit a signal on @p channel. All slot of every module connected to the specified channel in
     * will be called with the data @p value
     */
    template<typename ...ChannelTs>
    void emit (const Channel &channel, ChannelTs &&...value);

    /**
     * @brief As @see Module::emit (const Channel &channel,...) but it resolves the
     * channel name (small overhead for name resolution)
     */
    template<typename ...ChannelTs>
    void emit (const std::string &channel, ChannelTs &&...value);

    /**
     * @brief Emit a signal on a @p channel that has non-void return value. The principle
     * is the same as @see Module::emit, but only one slot can be connected
     * (@see Module::requestConnection will fail when trying to connect to
     * a channel to which another slot is already connected)
     */
    template<typename ReturnT, typename ...ChannelTs>
    ReturnT callService (const Channel &channel, ChannelTs &&...value);

    template<typename ReturnT, typename ...ChannelTs>
    ReturnT callService (const std::string &channel, ChannelTs &&...value);

    /**
     * @brief Handle to centralized Modflow @see ResourceManager
     */
    const ResourceManager &resources () const;
    ResourceManager &resources ();

private:
    void setEnabled (ChannelId enablingChannelid);

protected:
    std::shared_ptr<Modflow> _modflow;
    Event::SharedPtr _lastEvent;

private:
    std::set<ChannelId> _disablingChannels;
    std::string _name;
    bool _automaticParamUpdate;
};

/**
 * @brief Sources are a particular Module whose channels are declared by the
 * parent object, which emits signals on such channels externally.
 * The Modflow object will automatically load a module, available externally
 * via @ref Modflow::sources
 * @ingroup modflow
 */
class Sources final : public Module
{
public:
    NLIB_DEF_SMARTPTR(Sources)

protected:
    /// @brief @see Module::Module
    Sources (const std::shared_ptr<Modflow> &modflow):
        Module (modflow, "sources")
    {}

    /// @brief This module cannot be overridden, so no parameters can be associated
    /// to it
    void initParams (const NlParams::SharedPtr &params) override {}

    /// @brief Channel are created externally via @ref Sources::declareSource
    void setupNetwork () override {}

    /**
     * @brief Externally create a channel from the parent object of types @p ...ChannelTs
     */
    template<typename ...ChannelTs>
    Channel declareSourceChannel (const std::string &name);

    /**
     * @brief Emit an event from the parent object on channel @p channel with data @p ...args
     */
    template<typename ...ChannelTs>
    void callSource (const Channel &channel,
                    ChannelTs &&...args);
    /**
     * @brief As Sources::callSources (const Channel &...) but resolves the channel @p name
     */
    template<typename ...ChannelTs>
    void callSource (const std::string &name,
                    ChannelTs &&...args);

private:
    using Module::requestConnection;
};

/**
 * @brief Sinks is a default module that does not create regular channels,
 * but the Parent can create "sink" channels, to which Modules can regularly
 * emit signals, but they are connected to the external parent's callback
 */
class Sinks final : public Module
{
public:
    NLIB_DEF_SMARTPTR(Sinks)

    Sinks (const std::shared_ptr<Modflow> &modflow):
        Module (modflow, "sinks")
    {}

    /// @brief No network to setup
    void setupNetwork () override {}
    /// @brief No need for parameters
    void initParams (const NlParams::SharedPtr &params) override {}

    /**
     * @brief Create a special channel marked as "sink", and connect signals
     * emitted on such channel to @p parentCallback
     */
    template<typename ...ChannelTs, typename Callback>
    void declareSink (const std::string &name,  const Callback &parentCallback);

private:
    using Module::createChannel;
};

/**
 * @brief Type erasure wrapper for storing generic functions. For internal use
 * @ingroup modflow
 */
class SerializedSlot
{
    using SerializedFcn = std::function<std::any (const Event::SharedPtr &, const std::any &)>;
    
    template<typename ReturnT, typename ...SlotTs>
    std::enable_if_t<(sizeof ...(SlotTs) <= 1) &&
                     !std::is_same_v<ReturnT, void>>
    serialize (const Slot<ReturnT, SlotTs...> &slot);
    
    template<typename ReturnT, typename ...SlotTs, std::size_t ...is>
    std::enable_if_t<(sizeof ...(SlotTs) > 1) &&
                     !std::is_same_v<ReturnT, void>>
    serialize (const Slot<ReturnT, SlotTs...> &slot,
              std::index_sequence<is...>);
    
    template<typename ...SlotTs>
    std::enable_if_t<(sizeof ...(SlotTs) <= 1)>
    serialize (const Slot<void, SlotTs...> &slot);
    
    template<typename ...SlotTs, std::size_t ...is>
    std::enable_if_t<(sizeof ...(SlotTs) > 1)>
    serialize (const Slot<void, SlotTs...> &slot,
              std::index_sequence<is...>);
    
public:
    template<typename ReturnT, typename ...SlotTs>
    SerializedSlot (const Slot<ReturnT, SlotTs...> &slt,
                   const Channel &channel,
                   const std::string &slotName,
                   std::enable_if_t<(sizeof ...(SlotTs) <= 1)> * = nullptr);
    
    template<typename ReturnT, typename ...SlotTs>
    SerializedSlot (const Slot<ReturnT, SlotTs...> &slt,
                   const Channel &channel,
                   const std::string &slotName,
                   std::enable_if_t<(sizeof ...(SlotTs) > 1)> * = nullptr);
    
    template<typename ReturnT, typename SlotT>
    std::enable_if_t<std::is_same_v<ReturnT, void>, ReturnT>
    invoke (const Event::SharedPtr &event, SlotT &&arg) const;
    
    template<typename ReturnT, typename ...SlotTs>
    std::enable_if_t<(sizeof ...(SlotTs) > 1) &&
                         std::is_same_v<ReturnT, void>, ReturnT>
    invoke (const Event::SharedPtr &event, SlotTs &&...args) const;
    
    template<typename ReturnT>
    std::enable_if_t<std::is_same_v<ReturnT, void>, ReturnT>
    invoke (const Event::SharedPtr &event) const;
    
    template<typename ReturnT, typename SlotT>
    std::enable_if_t<!std::is_same_v<ReturnT, void>, ReturnT>
    invoke (const Event::SharedPtr &event, SlotT &&arg) const;
    
    template<typename ReturnT, typename ...SlotTs>
    std::enable_if_t<(sizeof ...(SlotTs) > 1) &&
                         !std::is_same_v<ReturnT, void>, ReturnT>
    invoke (const Event::SharedPtr &event, SlotTs &&...args) const;
    
    template<typename ReturnT>
    std::enable_if_t<!std::is_same_v<ReturnT, void>, ReturnT>
    invoke (const Event::SharedPtr &event) const;
    
    const std::string &name () const { return _name; }
    
private:
    std::string _name;
    Channel _channel;
    SerializedFcn _serialized;
};

/**
 * @brief This is the main class that handles the call flow between modules.
 * First, you will need to derive this class and override @ref loadModules, calling
 * @ref loadModule with the type of each module you want to load as template parameter.
 * Then:
 * - Create Modflow::SharedPtr with Modflow::create, with your Modflow derived class template
 * - Initialize the modflow object with @ref init, which will initialize sources and sinks modules
 * - Declare source channels via @ref sources() -> @ref Sources::declareSourceChannel, to create entry point channels
 * - Declare sink channels via @ref sinks() -> @ref Sinks::declareSink, to connect exit points to external callbacks
 * - Finalize the Modflow initialization via @ref finalize (), which will configure each module by colling @ref Module::initParams and @ref Module::setupNetwork, that each module shall override.
 * The initialization of the modules is done in the same order they have been loaded.
 *
 * After initialization, sources can be triggered by calling @ref sources() -> @ref Sources::callSource, which will
 * emit signals on regular channels as defined by @ref Sources::declareSourceChannels.
 * Modules connected to such channels will receive a function call on its associated
 * slot, which can in turn emit other signals on other channels, which can then be connected
 * to other modules or to sinks. Differently from regular channels, sink channels
 * are connected to an external callback that acts as exit point of Modflow.
 * @ingroup modflow
 */
class Modflow : public std::enable_shared_from_this<Modflow>
{
    using Connection = std::vector<SerializedSlot>;

protected:
    // Modflow must be created as shared pointer via "create()"
    Modflow ();

public:
    NLIB_DEF_SMARTPTR(Modflow)

    /**
     * @brief A Modflow object must be created via this method, which creates a shared
     * pointer, instantiated with the derived class type supplied as template parameter.
     */
    template<class DerivedModflow>
    static std::enable_if_t<std::is_base_of_v<Modflow, DerivedModflow>,
                            Modflow::SharedPtr>
    create ();

    /**
     * @brief This is the first function to be called. It loads sources and sink modules.
     * After that, it calls the virtual function @ref loadModules, that each Module derived
     * class must override, that is supposed to load all the modules, in the provided order.
     * @p nlParams is the overall parameter server. If initialized with nullptr all
     * parameter-related methods call will be disabled
     */
    void init (const NlParams::SharedPtr &nlParams = nullptr);

    /**
     * @brief To be called after the declaration of sources and sinks. For each module
     * in the same order as specified in @ref setupNetwork, it calls @ref Module::initParams
     * (if a NlParams parameter server has been supplied) and then @ref Module::setupNetwork
     */
    void finalize ();

    /// @brief Get a pointer to the source module object
    Sources::SharedPtr sources ();
    /// @brief Get a pointer to the sinks module object
    Sinks::SharedPtr sinks ();

    /**
     * @brief From the full parameter path, it extracts, the module name, finds the
     * module in the module list, and calls @ref Module::onParameterChange with the rest of
     * the parameter path.
     */
    void onParameterChange (const std::string &fullPath, const NlParams::SharedPtr &nlParams);

protected:
    friend class Module;
    friend class Sources;
    friend class Sinks;

    /**
     * @brief Construct a dynamically allocated object of type @p DerivedModule,
     * as shared pointer, and stores into Modflow's modules list.
     * @return Pointer to created module
     */
    template<class DerivedModule, typename ...Args>
    typename DerivedModule::SharedPtr
    loadModule (Args &&...args);

    /**
     * @brief Method to be overridden to specify the modules to be loaded via @ref loadModule
     */
    virtual void loadModules () = 0;

    /**
     * @brief Declare a new channel of types @p ...ChannelTs. The channel is owned by module @p owner,
     * that is the only module that can emit events on this channel.
     * @return A new @ref Channel object with the created channel information
     */
    template<typename ...ChannelTs>
    Channel createChannel(const std::string &name,
                          const Module *owner,
                          bool isSink = false);

    /**
     * @brief Get a @ref Channel object given its name. @par Complexity Logarithmic in the number of channels
     */
    Channel resolveChannel (const std::string &name);

    /**
     * @brief Create a connection named @p name from @p channel to a @p slot function.
     * @note Do not call this method directly. Use Module::requestConnection or Sinks::declareSink
     */
    template<typename ReturnT, typename ...ChannelTs>
    void createConnection (const Channel &channel,
                          const Slot<ReturnT, ChannelTs...> &slot,
                          const std::string &name);

    /**
     * @brief Emit a signal on @p channel. This will call the slot methods associated
     * to the channel of each module, supplying @p ...args.
     * @return If ReturnT is not void, forwards the return value of @p slot. In this case
     * only one slot can be connected per channel
     */
    template<typename ReturnT, typename ...ChannelTs>
    std::enable_if_t<std::is_same_v<ReturnT, void>, ReturnT>
    emit (const Channel &channel, const Module *caller, ChannelTs &&...args);

    template<typename ReturnT, typename ...ChannelTs>
    std::enable_if_t<!std::is_same_v<ReturnT, void>, ReturnT>
    emit (const Channel &channel, const Module *caller, ChannelTs &&...args);

    template<typename ReturnT, typename ...ChannelTs>
    ReturnT emit (const std::string &channelname, const Module *caller, ChannelTs &&...args);

private:
    bool hasParams () const { return _nlParams != nullptr; }

    template<typename ReturnT, typename ...ChannelTs>
    Event::SharedPtr prepareEmit (const Channel &channel, const Module *caller);
    void initDebugConfiguration ();
    bool debugFilters (const Event::SharedPtr &event);

protected:
    NlParams::SharedPtr _nlParams;
    ResourceManager _resources;

private:
    struct DebugConfiguration {
        bool enabled;
        bool filterOnly;
        std::vector<std::string> filterOnlyChannels;
        std::vector<std::string> filterExcludeChannels;
        std::vector<std::string> filterOnlyModules;
        std::vector<std::string> filterExcludeModules;
    } _debug;


    ChannelId _channelsSeq;
    Sources::SharedPtr _sources;
    Sinks::SharedPtr _sinks;
    std::vector<Module::SharedPtr> _modules;
    std::unordered_map<std::string, Channel> _channelNames;
    std::vector<Connection> _connections;
};




}

// Template implementations
#ifndef NL_MODFLOW_IMPL_HPP
#include "nl_modflow_impl.hpp"
#endif

#endif // NL_MODFLOW_H
