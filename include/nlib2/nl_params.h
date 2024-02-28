#ifndef NL_PARAMS_H
#define NL_PARAMS_H

#include "nl_utils.h"
#include <rclcpp/exceptions.hpp>
#include <functional>
#include <optional>
#include <any>
#include <typeindex>

namespace nlib2 {

struct IParameterServer
{
    NLIB_DEF_SMARTPTR(IParameterServer)

    template<typename ParameterT>
    void declareParameter (const std::string &name, const std::optional<ParameterT> &defaultValue);
    template<typename ParameterT>
    ParameterT getParameter (const std::string &name);

    virtual ~IParameterServer () = default;

protected:
    virtual void declareParameter (const std::string &name, const std::optional<std::any> &defaultValue, const std::type_index &type) = 0;
    virtual std::any getParameter (const std::string &name, const std::type_index &type) = 0;
};

class NlParams : public std::enable_shared_from_this<NlParams>
{
public:
    NLIB_DEF_SMARTPTR(NlParams)

private:
    NlParams (const std::string &paramNamespace, const NlParams::SharedPtr &parent, const IParameterServer::SharedPtr &parameterServer):
        _parent(parent),
        _paramNamespace(paramNamespace),
        _parameterServer(parameterServer)
    {
    }

    NlParams (const std::shared_ptr<IParameterServer> &parameterServer):
        _parent(nullptr),
        _paramNamespace(""),
        _parameterServer(parameterServer)
    {
    }

public:
    static NlParams::SharedPtr initialize (const IParameterServer::SharedPtr &parameterServer);

    template<typename ParameterT>
    ParameterT get (const std::string &name);

    template<typename ParameterT>
    void declare (const std::string &name, const std::optional<ParameterT> &defaultValue = std::nullopt);

    template<typename ParameterT>
    ParameterT declareAndGet (const std::string &name, const std::optional<ParameterT> &defaultValue = std::nullopt);

    NlParams::SharedPtr derive (const std::string &childName);

//private:
    std::string fullName (const std::string &childName);


protected:
    NlParams::SharedPtr _parent;
    std::string _paramNamespace;
    IParameterServer::SharedPtr _parameterServer;
};


}


#ifndef NL_PARAMS_IMPL_HPP
#include "nl_params_impl.hpp"
#endif

#endif // NL_PARAMS_H
