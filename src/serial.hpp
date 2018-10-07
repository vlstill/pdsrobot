#pragma once

namespace axx {

struct Serial {
    Serial &operator<<( const char *v );
    Serial &operator<<( char v );

    Serial &operator>>( char &v );
};

extern Serial serial;

}
