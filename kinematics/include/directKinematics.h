#ifndef DIRECTKINEMATICS_H
#define DIRECTKINEMATICS_H

#include "typedefs.h"

/**
 * @brief  Comutation of the direkt kinematics for the UR5 robotic arm.
 *
 * @param Th The vector x6 of joint angles.
 * @return directK s struct with the position and the rotation of the end effector.
 */
directK ur5DirectKinematics(Vector6d Th);

#endif