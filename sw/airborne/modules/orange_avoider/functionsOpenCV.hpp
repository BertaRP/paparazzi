/*
 * functionsOpenCV.hpp
 *
 *  Created on: Mar 16, 2017
 *      Author: mavlabcourse
 */

#ifndef SW_AIRBORNE_MODULES_ORANGE_AVOIDER_FUNCTIONSOPENCV_HPP_
#define SW_AIRBORNE_MODULES_ORANGE_AVOIDER_FUNCTIONSOPENCV_HPP_

#ifdef __cplusplus
extern "C" {
#endif

extern int myfunction(int, void *ob);
extern Ptr<BackgroundSubtractorMOG2> createBackgroundSubtractorMOG2(int, double, bool);

#ifdef __cplusplus
}
#endif

#include "functionsOpenCV.hpp"


Ptr<BackgroundSubtractorMOG2> createBackgroundSubtractorMOG2(int _history, double _varThreshold,
                                                             bool _bShadowDetection)
{
    return makePtr<BackgroundSubtractorMOG2Impl>(_history, (float)_varThreshold, _bShadowDetection);
}



#endif /* SW_AIRBORNE_MODULES_ORANGE_AVOIDER_FUNCTIONSOPENCV_HPP_ */
