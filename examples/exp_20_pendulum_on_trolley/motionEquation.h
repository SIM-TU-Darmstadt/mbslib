#ifndef MOTION_EQUATION_H
#define MOTION_EQUATION_H

void lib_init(double l, double m, double k, double d, double g);
void motionEquation(const double * x, double * xp);
void kinematics(const double * x, double * pos, double * vel);
#ifdef USE_ADOLC
void derivatives(const double * x, double * results);
#endif
void lib_cleanup();

#endif
