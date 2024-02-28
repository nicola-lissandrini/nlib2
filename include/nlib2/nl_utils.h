#ifndef NL_UTILS_H
#define NL_UTILS_H

/** @file nlutils.h
 *  @author Nicola Lissandrini
 */

#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <ostream>
#include <string>
#include <memory>
#include <cxxabi.h>
#include <unistd.h>
#include <chrono>

namespace nlib2 {

/// @brief Shortcut to cout line
/// @ingroup qdt
#define COUT(str) {std::cout <<  (str) << std::endl;}
/// @brief Print name and value of expression
/// @ingroup qdt
#define COUTN(var) {std::cout << "\e[33m" << #var << "\e[0m" << std::endl << var << std::endl;}
/// @brief Print name and value of an expression that is a time point
/// @ingroup qdt
#define COUTNT(var) {std::cout << "\e[33m" << #var << "\e[0m" << std::endl << nlib::printTime(var) << std::endl;}
/// @brief Print name and shape of expressione
/// @ingroup qdt
#define COUTNS(var) {COUTN(var.sizes());}
/// @brief Print calling function, name and value of expression
/// @ingroup qdt
#define COUTNF(var) {std::cout << "\e[32m" << __PRETTY_FUNCTION__ << "\n\e[33m" << #var << "\e[0m" << std::endl << var << std::endl;}
/// @brief Shortcut for printing name and value and returning the result of expression
/// @ingroup qdt
#define COUT_RET(var) {auto __ret = (var); COUTN(var); return __ret;}
#define DEMANGLE(type_info) (abi::__cxa_demangle(type_info.name(), NULL,NULL,NULL))
/// @brief Get demangled type of expression
/// @ingroup qdt
#define TYPE(type) (DEMANGLE(typeid(type)))
/// @brief Shortcut for printing boost stacktrace
/// @ingroup qdt
#define STACKTRACE {std::cout << boost::stacktrace::stacktrace() << std::endl;}
/// @brief Print calling function and current file and line
/// @ingroup qdt
#define QUA {std::cout << "\e[33mReached " << __PRETTY_FUNCTION__ << "\e[0m:" << __LINE__ << std::endl; }

#define WAIT_GDB {volatile int __done = 0; while (!__done) sleep(1);}
template<typename T>
std::string getFcnName (T functionAddress) {
    Dl_info info;
    dladdr (reinterpret_cast<const void *>(functionAddress), &info);
    return abi::__cxa_demangle(info.dli_sname, NULL, NULL, NULL);
}


/**
 * @section Shared ptr tools
 */

/// @brief define nested Classname::Ptr symbol for shared_ptr of the class
/// To be used inside class definition
#define __NLIB_DEF_PTR(classname, alias, ptr_type) \
using alias##Ptr = std::ptr_type<classname>; \
using Const##alias##Ptr = std::ptr_type<const classname>;

#define __NLIB_MAKE_PTR(classname, ptr_type) \
template<typename ...Args> \
static std::ptr_type##_ptr<classname> make_##ptr_type (Args && ...args) { \
        return std::make_##ptr_type<classname> (std::forward<Args> (args) ...); \
}

#define __NLIB_DEF_SHARED(classname) \
__NLIB_DEF_PTR(classname, Shared, shared_ptr) \
__NLIB_MAKE_PTR(classname, shared)

#define __NLIB_DEF_UNIQUE(classname) \
__NLIB_DEF_PTR(classname, Unique, unique_ptr) \
__NLIB_MAKE_PTR(classname, unique)

#define __NLIB_DEF_WEAK(classname) \
__NLIB_DEF_PTR(classname, Weak, weak_ptr)

#define NLIB_DEF_SMARTPTR(classname) \
__NLIB_DEF_SHARED(classname) \
__NLIB_DEF_UNIQUE(classname) \
__NLIB_DEF_WEAK(classname)

#define NLIB_CRTP_DERIVED(Derived) \
Derived &derived () { return static_cast<Derived&> (*this); } \
const Derived &derived () const { return static_cast<const Derived &> (*this); }


inline void quickGDB (int argc, char *argv[]) {
    if (argc == 1)
        return;
    if (std::strcmp(argv[1], "true") == 0) {
        std::cout << "Waiting for gdb to attach..." << std::endl;
        WAIT_GDB;
    }
}

template<class to_duration = std::chrono::milliseconds, class time_point>
std::string printTime (const time_point &time) {
    using TimeInt = std::chrono::time_point<typename time_point::clock>;
    TimeInt timeInt = std::chrono::time_point_cast<typename TimeInt::duration> (time);
    auto coarse = std::chrono::system_clock::to_time_t(timeInt);
    auto fine = std::chrono::time_point_cast<to_duration>(time);

    char buffer[sizeof "9999-12-31 23:59:59.999"];

    std::snprintf(buffer + std::strftime(buffer, sizeof buffer - 3,
                                         "%F %T.", std::localtime(&coarse)),
                  4, "%03lu", fine.time_since_epoch().count() % 1000);
    return buffer;
}

} // namespace nlib2

#endif // NL_UTILS_H
