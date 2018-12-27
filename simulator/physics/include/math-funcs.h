#ifndef     MATH_FUNCS_H
#define     MATH_FUNCS_H

#include    <cmath>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T hs_p(T x)
{
    if ( x > static_cast<T>(0) )
        return static_cast<T>(1);
    else
        return static_cast<T>(0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
bool hs_p(T x)
{
    return x > static_cast<T>(0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T hs_n(T x)
{
    if ( x < static_cast<T>(0) )
        return static_cast<T>(1);
    else
        return static_cast<T>(0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
bool hs_n(T x)
{
    return x < static_cast<T>(0);
}

#endif // MATH_FUNCS_H