/*
 * CubeReading.hpp
 *
 *  Created on: 19-11-2010
 *      Author: przemo
 */

#ifndef CUBEREADING_HPP_
#define CUBEREADING_HPP_

#include "Reading.hpp"

namespace Types {
namespace Mrrocpp_Proxy {

/**
 *
 */
class CubeReading: public Reading
{
public:
	CubeReading()
	{
		exist=false;
	}

	virtual ~CubeReading()
	{
	}

	virtual CubeReading* clone()
	{
		return new CubeReading(*this);
	}
	bool exist;
	int color[3][3][3];

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}

private:
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
//		LOG(LWARNING) << "CubeReading::serialize()\n";
		ar & boost::serialization::base_object <Reading>(*this);

		ar & exist;
		ar & color;
	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types

#endif /* CUBEREADING_HPP_ */
