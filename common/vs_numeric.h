#ifndef __VS_NUMERIC_H__
#define __VS_NUMERIC_H__

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_2PI
#define M_2PI     6.28318530717958647692
#endif

/** \brief y = sign(x)
\return 1 if x>=0; -1 if x<0
*/
inline int sign(double d){
	return (d >= 0) ? 1 : -1;
}

/** \brief clip val to interval [min_val,max_val]
\param[in] val clipped number
\param[in] min_val min value of interval
\param[in] max_val max value of interval
\return min_val if val<min_val; max_val if val>max_val; val else
*/
template<class T>
inline T clip(T val, T min_val, T max_val){
	return val<min_val ? min_val : (val>max_val ? max_val : val);
}


/** \brief Normalize angle in rad to range (-pi,pi] 
	\param[in] angle input angle in rad
	\return normalized angle in (-pi,pi] equal to input angle
*/
inline double normalizeRad(double angle){
	angle = fmod(angle, M_2PI);
	if (angle <= -M_PI) { angle += M_2PI; }
	else if (angle > M_PI){ angle -= M_2PI; }
	return angle;
}

/** \brief Normalize angle in deg to range (-180,180]
	\param[in] angle input angle in deg
	\return normalized angle in (-180,180] equal to input angle
*/
inline double normalizeDeg(double theta){
	theta = fmod(theta, 360);
	if (theta <= -180) { theta += 360; }
	else if (theta > 180){ theta -= 360; }
	return theta;
}

/** \brief convert angle from unit deg to unit rad
	\param[in] angle input angle in deg
	\return angle in rad equivalent to input angle
*/
inline double deg2rad(double angle){
	return angle*0.017453292519943;
}

/** \brief convert angle from unit rad to unit deg
	\param[in] angle input angle in rad
	\return angle in deg equivalent to input angle
*/
inline double rad2deg(double angle){
	return angle*57.295779513082;
}

/** \brief judge x==y ? 	*/
inline bool floaEqual(double x, double y){
	return fabs(x - y) < 1e-5;
}

/** \brief Solve linear equation y=ax+b. Given 2 points (x1,y1),(x2,y2) in line and x, find the y in line corresponding to x.
	\param[in] x1 x of first point(x1,y1)
	\param[in] y1 y of first point(x1,y1)
	\param[in] x2 x of second point(x2,y2)
	\param[in] y2 y of second point(x2,y2)
	\param[in] x the x to be solved
	\return y y coresponding to x. (y-y1)/(x-x1) = (y2-y1)/(x2-x1) = (y-y2)/(x-x2);
*/
inline double linearInterpolation(double x1, double y1, double x2, double y2, double x){
	if (floaEqual(x1,x2)) {
		printf("[ERROR] linearInterpolation: assert x1 != x2\n");
		return 0;
	}
	return (y2 - y1) / (x2 - x1)*(x - x1) + y1;
}

#endif