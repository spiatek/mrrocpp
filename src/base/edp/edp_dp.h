/*!
 * \file edp_dp.h
 * \brief File containing the declaration of edp data port class
 *
 * \author yoyek
 *
 */

#ifndef __EDP_DP_H
#define __EDP_DP_H

#include <boost/thread/mutex.hpp>

namespace mrrocpp {
namespace edp {
namespace common {

template <class T>
class edp_dp
{
protected:

	boost::mutex boost_mutex; // mutex do sily   XXXXXX

	/**
	 * @brief data stored basing on template
	 */
	T data;

public:

	/**
	 * @brief Constructor
	 */
	edp_dp()
	{
	}

	void write(const T &input)
	{
		boost::mutex::scoped_lock lock(boost_mutex);
		data = input;
	}

	void read(T &output)
	{
		boost::mutex::scoped_lock lock(boost_mutex);
		output = data;
	}

	T read()
	{
		boost::mutex::scoped_lock lock(boost_mutex);
		return data;
	}

};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
