#ifndef __VS_TRANSFORM_H__
#define __VS_TRANSFORM_H__

/** conversion between quaternion(w,x,y,z), eular(yaw,pitch,roll) and rotation matrix
*/


void quat2rot(const float* q, float *r);

void quat2euler(const float* q, float *e);

void euler2rot(const float* e, float *r);

void euler2quat(const float* e, float *q);

void rot2quat(const float* r, float *q);

void rot2euler(const float* r, float *e);


#endif