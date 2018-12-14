#pragma once

#include <string>
#include <stdexcept>
#include <iostream>

struct Die
{
	Die( std::string msg ) : msg( msg ) { }

	~Die() {
		if ( triggered ) {
			// throw std::runtime_error( msg ); // TODO
			std::cerr << "ERROR: " << msg << std::endl << std::flush;
			while ( true ) { }
//			std::abort();
		}
	}

	friend Die operator||( esp_err_t ret, Die &&die ) {
		if ( ret != ESP_OK )
			die.triggered = true;
		return std::move( die );
	}

  private:
	bool triggered = false;
	std::string msg;
};
