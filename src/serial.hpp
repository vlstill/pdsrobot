#pragma once

namespace axx {

struct Serial {
    Serial &operator<<( const char *v );
    Serial &operator<<( char v );

    Serial &operator<<( short v );
    Serial &operator<<( int v );
    Serial &operator<<( long v );

    Serial &operator<<( unsigned short v );
    Serial &operator<<( unsigned v );
    Serial &operator<<( unsigned long v );

    Serial &operator>>( char &v );
};

extern Serial serial;

}
