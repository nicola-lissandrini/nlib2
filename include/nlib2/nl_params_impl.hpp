#ifndef NL_PARAMS_IMPL_HPP
#define NL_PARAMS_IMPL_HPP

#ifndef NL_PARAMS_H
#include "nl_params.h"
#endif

namespace nlib2 {

template<typename ParameterT>
void IParameterServer::declareParameter (const std::string &name, const std::optional<ParameterT> &defaultValue) {
    if (defaultValue.has_value()) // handle conversion from std::optional<ParameterT> to std::optional<std::any>
        declareParameter (name, *defaultValue, typeid(ParameterT));
    else
        declareParameter (name, std::nullopt, typeid(ParameterT));
}

template<typename ParameterT>
ParameterT IParameterServer::getParameter (const std::string &name) {
    return std::any_cast<ParameterT> (getParameter (name, typeid(ParameterT)));
}


inline std::string NlParams::fullName(const std::string &childName) {
    return _parent == nullptr ? childName : _paramNamespace + "." + childName;
}

inline NlParams::SharedPtr NlParams::derive(const std::string &childName) {
    return NlParams::SharedPtr (new NlParams (fullName (childName), shared_from_this(), _parameterServer));
}

template<typename ParameterT>
ParameterT NlParams::declareAndGet(const std::string &name, const std::optional<ParameterT> &defaultValue) {
    try {
        declare<ParameterT> (name, defaultValue);
    } catch (...) {
        // If declaration fails just ignore, potential issues are caught by "get"
    }
    return get<ParameterT> (name);
}

template<typename ParameterT>
void NlParams::declare (const std::string &name, const std::optional<ParameterT> &defaultValue) {
    _parameterServer->declareParameter<ParameterT> (fullName (name), defaultValue);
}

template<typename ParameterT>
ParameterT NlParams::get(const std::string &name) {
    return _parameterServer->getParameter<ParameterT> (fullName (name));
}


inline NlParams::SharedPtr NlParams::initialize (const IParameterServer::SharedPtr &parameterServer) {
    return NlParams::SharedPtr (new NlParams (parameterServer));
}

}

#endif // NL_PARAMS_IMPL_HPP
