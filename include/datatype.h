#include <iostream>

#ifndef _DATATYPE_H_
#define _DATATYPE_H_

namespace datatype
{
    typedef union USHORT_UNION_{
        uint16_t ushort_;
        uint8_t bytes_[2];
    } USHORT_UNION;

    typedef union FLOAT_UNION_{
        float float_;
        char bytes_[4];   
    } FLOAT_UNION;
};


#endif _DATATYPE_H_