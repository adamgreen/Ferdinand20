/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Fake out enough of the mbed library to get my AdafruitPrecision9DoF code working with the Nordic SDK.
#ifndef MBED_FAKE_H_
#define MBED_FAKE_H_

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <new>
#include <app_error.h>
#include <nrf.h>
#include <nrf_delay.h>
#include <nrf_drv_twi.h>
#include <nrf_drv_gpiote.h>


typedef uint32_t PinName;

#define MBED_ASSERT assert

/** MBED_STATIC_ASSERT
 *  Declare compile-time assertions, results in compile-time error if condition is false
 *
 *  The assertion acts as a declaration that can be placed at file scope, in a
 *  code block (except after a label), or as a member of a C++ class/struct/union.
 *
 *  @note
 *  Use of MBED_STATIC_ASSERT as a member of a struct/union is limited:
 *  - In C++, MBED_STATIC_ASSERT is valid in class/struct/union scope.
 *  - In C, MBED_STATIC_ASSERT is not valid in struct/union scope, and
 *    MBED_STRUCT_STATIC_ASSERT is provided as an alternative that is valid
 *    in C and C++ class/struct/union scope.
 *
 *  @code
 *  MBED_STATIC_ASSERT(MBED_LIBRARY_VERSION >= 120,
 *          "The mbed library must be at least version 120");
 *
 *  int main() {
 *      MBED_STATIC_ASSERT(sizeof(int) >= sizeof(char),
 *              "An int must be larger than a char");
 *  }
 *  @endcode
 */
#if defined(__cplusplus) && (__cplusplus >= 201103L || __cpp_static_assert >= 200410L)
#define MBED_STATIC_ASSERT(expr, msg) static_assert(expr, msg)
#elif !defined(__cplusplus) && __STDC_VERSION__ >= 201112L
#define MBED_STATIC_ASSERT(expr, msg) _Static_assert(expr, msg)
#elif defined(__cplusplus) && defined(__GNUC__) && defined(__GXX_EXPERIMENTAL_CXX0X__) \
    && (__GNUC__*100 + __GNUC_MINOR__) > 403L
#define MBED_STATIC_ASSERT(expr, msg) __extension__ static_assert(expr, msg)
#elif !defined(__cplusplus) && defined(__GNUC__) && !defined(__CC_ARM) \
    && (__GNUC__*100 + __GNUC_MINOR__) > 406L
#define MBED_STATIC_ASSERT(expr, msg) __extension__ _Static_assert(expr, msg)
#elif defined(__ICCARM__)
#define MBED_STATIC_ASSERT(expr, msg) static_assert(expr, msg)
#else
#define MBED_STATIC_ASSERT(expr, msg) \
    enum {MBED_CONCAT(MBED_ASSERTION_AT_, __LINE__) = sizeof(char[(expr) ? 1 : -1])}
#endif

/** Callback class based on template specialization
 *
 * @Note Synchronization level: Not protected
 */
template <typename F>
class Callback;

// Internal sfinae declarations
//
// These are used to eliminate overloads based on type attributes
// 1. Does a function object have a call operator
// 2. Does a function object fit in the available storage
//
// These eliminations are handled cleanly by the compiler and avoid
// massive and misleading error messages when confronted with an
// invalid type (or worse, runtime failures)
namespace detail {
    struct nil {};

    template <bool B, typename R = nil>
    struct enable_if { typedef R type; };

    template <typename R>
    struct enable_if<false, R> {};

    template <typename M, M>
    struct is_type {
        static const bool value = true;
    };
}

/** Callback class based on template specialization
 *
 * @Note Synchronization level: Not protected
 */
template <typename R>
class Callback<R()> {
public:
    /** Create a Callback with a static function
     *  @param func     Static function to attach
     */
    Callback(R (*func)() = 0) {
        if (!func) {
            _ops = 0;
        } else {
            generate(func);
        }
    }

    /** Attach a Callback
     *  @param func     The Callback to attach
     */
    Callback(const Callback<R()> &func) {
        if (func._ops) {
            func._ops->move(this, &func);
        }
        _ops = func._ops;
    }

    /** Create a Callback with a member function
     *  @param obj      Pointer to object to invoke member function on
     *  @param method   Member function to attach
     */
    template<typename T, typename U>
    Callback(U *obj, R (T::*method)()) {
        generate(method_context<T, R (T::*)()>(obj, method));
    }

    /** Create a Callback with a member function
     *  @param obj      Pointer to object to invoke member function on
     *  @param method   Member function to attach
     */
    template<typename T, typename U>
    Callback(const U *obj, R (T::*method)() const) {
        generate(method_context<const T, R (T::*)() const>(obj, method));
    }

    /** Create a Callback with a member function
     *  @param obj      Pointer to object to invoke member function on
     *  @param method   Member function to attach
     */
    template<typename T, typename U>
    Callback(volatile U *obj, R (T::*method)() volatile) {
        generate(method_context<volatile T, R (T::*)() volatile>(obj, method));
    }

    /** Create a Callback with a member function
     *  @param obj      Pointer to object to invoke member function on
     *  @param method   Member function to attach
     */
    template<typename T, typename U>
    Callback(const volatile U *obj, R (T::*method)() const volatile) {
        generate(method_context<const volatile T, R (T::*)() const volatile>(obj, method));
    }

    /** Create a Callback with a static function and bound pointer
     *  @param func     Static function to attach
     *  @param arg      Pointer argument to function
     */
    template<typename T, typename U>
    Callback(R (*func)(T*), U *arg) {
        generate(function_context<R (*)(T*), T>(func, arg));
    }

    /** Create a Callback with a static function and bound pointer
     *  @param func     Static function to attach
     *  @param arg      Pointer argument to function
     */
    template<typename T, typename U>
    Callback(R (*func)(const T*), const U *arg) {
        generate(function_context<R (*)(const T*), const T>(func, arg));
    }

    /** Create a Callback with a static function and bound pointer
     *  @param func     Static function to attach
     *  @param arg      Pointer argument to function
     */
    template<typename T, typename U>
    Callback(R (*func)(volatile T*), volatile U *arg) {
        generate(function_context<R (*)(volatile T*), volatile T>(func, arg));
    }

    /** Create a Callback with a static function and bound pointer
     *  @param func     Static function to attach
     *  @param arg      Pointer argument to function
     */
    template<typename T, typename U>
    Callback(R (*func)(const volatile T*), const volatile U *arg) {
        generate(function_context<R (*)(const volatile T*), const volatile T>(func, arg));
    }

    /** Create a Callback with a function object
     *  @param func     Function object to attach
     *  @note The function object is limited to a single word of storage
     */
    template <typename F>
    Callback(F f, typename detail::enable_if<
                detail::is_type<R (F::*)(), &F::operator()>::value &&
                sizeof(F) <= sizeof(uintptr_t)
            >::type = detail::nil()) {
        generate(f);
    }

    /** Create a Callback with a function object
     *  @param func     Function object to attach
     *  @note The function object is limited to a single word of storage
     */
    template <typename F>
    Callback(const F f, typename detail::enable_if<
                detail::is_type<R (F::*)() const, &F::operator()>::value &&
                sizeof(F) <= sizeof(uintptr_t)
            >::type = detail::nil()) {
        generate(f);
    }

    /** Create a Callback with a function object
     *  @param func     Function object to attach
     *  @note The function object is limited to a single word of storage
     */
    template <typename F>
    Callback(volatile F f, typename detail::enable_if<
                detail::is_type<R (F::*)() volatile, &F::operator()>::value &&
                sizeof(F) <= sizeof(uintptr_t)
            >::type = detail::nil()) {
        generate(f);
    }

    /** Create a Callback with a function object
     *  @param func     Function object to attach
     *  @note The function object is limited to a single word of storage
     */
    template <typename F>
    Callback(const volatile F f, typename detail::enable_if<
                detail::is_type<R (F::*)() const volatile, &F::operator()>::value &&
                sizeof(F) <= sizeof(uintptr_t)
            >::type = detail::nil()) {
        generate(f);
    }


    /** Destroy a callback
     */
    ~Callback() {
        if (_ops) {
            _ops->dtor(this);
        }
    }

    /** Assign a callback
     */
    Callback &operator=(const Callback &that) {
        if (this != &that) {
            this->~Callback();
            new (this) Callback(that);
        }

        return *this;
    }

    /** Call the attached function
     */
    R call() const {
        MBED_ASSERT(_ops);
        return _ops->call(this);
    }

    /** Call the attached function
     */
    R operator()() const {
        return call();
    }

    /** Test if function has been attached
     */
    operator bool() const {
        return _ops;
    }

    /** Test for equality
     */
    friend bool operator==(const Callback &l, const Callback &r) {
        return memcmp(&l, &r, sizeof(Callback)) == 0;
    }

    /** Test for inequality
     */
    friend bool operator!=(const Callback &l, const Callback &r) {
        return !(l == r);
    }

    /** Static thunk for passing as C-style function
     *  @param func Callback to call passed as void pointer
     */
    static R thunk(void *func) {
        return static_cast<Callback*>(func)->call();
    }

private:
    // Stored as pointer to function and pointer to optional object
    // Function pointer is stored as union of possible function types
    // to garuntee proper size and alignment
    struct _class;
    union {
        void (*_staticfunc)();
        void (*_boundfunc)(_class*);
        void (_class::*_methodfunc)();
    } _func;
    void *_obj;

    // Dynamically dispatched operations
    const struct ops {
        R (*call)(const void*);
        void (*move)(void*, const void*);
        void (*dtor)(void*);
    } *_ops;

    // Generate operations for function object
    template <typename F>
    void generate(const F &f) {
        static const ops ops = {
            &Callback::function_call<F>,
            &Callback::function_move<F>,
            &Callback::function_dtor<F>,
        };

        MBED_STATIC_ASSERT(sizeof(Callback) - sizeof(_ops) >= sizeof(F),
                "Type F must not exceed the size of the Callback class");
        new (this) F(f);
        _ops = &ops;
    }

    // Function attributes
    template <typename F>
    static R function_call(const void *p) {
        return (*(F*)p)();
    }

    template <typename F>
    static void function_move(void *d, const void *p) {
        new (d) F(*(F*)p);
    }

    template <typename F>
    static void function_dtor(void *p) {
        ((F*)p)->~F();
    }

    // Wrappers for functions with context
    template <typename O, typename M>
    struct method_context {
        M method;
        O *obj;

        method_context(O *obj, M method)
            : method(method), obj(obj) {}

        R operator()() const {
            return (obj->*method)();
        }
    };

    template <typename F, typename A>
    struct function_context {
        F func;
        A *arg;

        function_context(F func, A *arg)
            : func(func), arg(arg) {}

        R operator()() const {
            return func(arg);
        }
    };
};

/** Create a callback class with type infered from the arguments
 *
 *  @param obj      Optional pointer to object to bind to function
 *  @param method   Member function to attach
 *  @return         Callback with infered type
 */
template<typename T, typename U, typename R>
Callback<R()> callback(U *obj, R (T::*method)()) {
    return Callback<R()>(obj, method);
}


static inline void wait_ms(int ms)
{
    nrf_delay_ms(ms);
}

class I2C
{
public:
    I2C(PinName sdaPin, PinName sclPin)
    {
        m_i2cInstance.reg.p_twim = NRF_TWIM0;
        m_i2cInstance.drv_inst_idx = TWI0_INSTANCE_INDEX;
        m_i2cInstance.use_easy_dma = true;

        nrf_drv_twi_config_t i2cConfig = {
            .scl                = sclPin,
            .sda                = sdaPin,
            .frequency          = NRF_TWI_FREQ_400K,
            .interrupt_priority = 7,
            .clear_bus_init     = TWI_DEFAULT_CONFIG_CLR_BUS_INIT,
            .hold_bus_uninit    = TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT
        };

        ret_code_t result = nrf_drv_twi_init(&m_i2cInstance, &i2cConfig, NULL, NULL);
        APP_ERROR_CHECK(result);
        nrf_drv_twi_enable(&m_i2cInstance);
    }

    void frequency(int hz)
    {
        // Hardcoded at 400kHz in constructor.
        assert ( hz = 400000 );
    }

    int read(int address, char *data, int length, bool repeated = false)
    {
        ret_code_t result = nrf_drv_twi_rx(&m_i2cInstance, address>>1, (uint8_t*)data, length);
        if (result != NRF_SUCCESS)
        {
            return -1;
        }
        return 0;
    }

    int write(int address, const char *data, int length, bool repeated = false)
    {
        ret_code_t result = nrf_drv_twi_tx(&m_i2cInstance, address>>1, (uint8_t const*)data, length, repeated);
        if (result != NRF_SUCCESS)
        {
            return -1;
        }
        return 0;
    }

protected:
    nrf_drv_twi_t   m_i2cInstance;
};

// Only works for 1 InterruptIn instantiation which is all I need to get this code working.
class InterruptIn
{
public:
    InterruptIn(PinName pin)
    {
        m_pin = pin;
    }

    void fall(Callback<void()> func)
    {
        s_interruptInFall = func;

        nrf_drv_gpiote_in_config_t gpioteConfig = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
        ret_code_t result = nrf_drv_gpiote_in_init(m_pin, &gpioteConfig, interruptInCallback);
        APP_ERROR_CHECK(result);
        nrf_drv_gpiote_in_event_enable(m_pin, true);
    }

protected:
    static void interruptInCallback(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
    {
        s_interruptInFall();
    }
    static Callback<void()> s_interruptInFall;

    nrf_drv_gpiote_pin_t    m_pin;
};

class Timer
{
public:
    Timer()
    {
    }

    void start()
    {
    }

    void reset()
    {
    }

    int read_us()
    {
        // These times are just going to be ignored anyway so return a fixed value.
        return 1;
    }
};


#endif /* MBED_FAKE_H_ */
