/**
 *
 * Copied from BYU fcu_io
 * \file serial_exception.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef SNATCH_SERIAL_EXCEPTION_H
#define SNATCH_SERIAL_EXCEPTION_H

#include <exception>
#include <string>
#include <sstream>

#include <boost/system/system_error.hpp>

namespace snatch {

class SnatchSerialException: public std::exception {
public:
	explicit SnatchSerialException(const char * const description) {
		init(description);
	}

	explicit SnatchSerialException(const std::string &description) {
		init(description.c_str());
	}

	explicit SnatchSerialException(const boost::system::system_error &err) {
		init(err.what());
	}

	SnatchSerialException(const SnatchSerialException &other) :
			what_(other.what_) {
	}

	~SnatchSerialException() throw () {
	}

	virtual const char* what() const throw () {
		return what_.c_str();
	}

private:
	std::string what_;

	void init(const char * const description) {
		std::ostringstream ss;
		ss << "Serial Error: " << description;
		what_ = ss.str();
	}
};

} // namespace snatch

#endif // SNATCH_SERIAL_EXCEPTION_H

