#ifndef BITCAST_H
#define BITCAST_H

#include <cstring>

template <class To, class From>
To portable_bit_cast(const From& src) {
    static_assert(sizeof(To) == sizeof(From), "size mismatch");
    
    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
}

#endif // BITCAST_H