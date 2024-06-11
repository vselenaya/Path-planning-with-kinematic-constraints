#pragma once

#define DEBUG

#ifdef DEBUG
    #define rassert(condition, info) if (!(condition)) { throw std::runtime_error("Assertion failed!\n" "Add info: " info); }
#else
    #define rassert(condition, info) ;
#endif