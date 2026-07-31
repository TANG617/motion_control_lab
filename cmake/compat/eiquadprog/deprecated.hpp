#pragma once

#if defined(__cplusplus) && __cplusplus >= 201402L
#define EIQUADPROG_DEPRECATED [[deprecated]]
#define EIQUADPROG_DEPRECATED_MESSAGE(message) [[deprecated(#message)]]
#elif defined(__GNUC__) || defined(__clang__)
#define EIQUADPROG_DEPRECATED __attribute__((deprecated))
#define EIQUADPROG_DEPRECATED_MESSAGE(message) __attribute__((deprecated(#message)))
#elif defined(_MSC_VER)
#define EIQUADPROG_DEPRECATED __declspec(deprecated)
#define EIQUADPROG_DEPRECATED_MESSAGE(message) __declspec(deprecated(#message))
#else
#define EIQUADPROG_DEPRECATED
#define EIQUADPROG_DEPRECATED_MESSAGE(message)
#endif
