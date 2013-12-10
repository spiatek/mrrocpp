/*!
 * \file
 * \brief
 * \author Przemysław Krajewski
 */

#ifndef WALLOFCUBE_HPP_
#define WALLOFCUBE_HPP_

#include <opencv2/imgproc/imgproc.hpp>

struct WallOfCube
{
	FieldOfCube field[4][4];
	bool exist;
};

#endif /* WALLOFCUBE_HPP_ */
