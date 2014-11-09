//Description:  This file defines all the functions that are used in the file.
// Name: Sway
// Time: 11/07/2014
// Organization: Institute of Automation, Chinese Academy of Sciences

//=====================================================================================================================//

#ifndef SW_FUNCTIONS_H
#define SW_FUNCTIONS_H

#include"sw_dataType.h"

////////////////////////////////////ANGle-AXIS METHOD///////////////////////////////////////////////
// get a angle and a axis from two vectors
// **input
// src-- vector
// dst-- vector

// **output
// axis-- the rotation axis
// angle-- the rotation angle
float angleAxisFromDirections(Vec3 & src, Vec3& dest, Vec3 & axis);

#endif // SW_FUNCTIONS_H
