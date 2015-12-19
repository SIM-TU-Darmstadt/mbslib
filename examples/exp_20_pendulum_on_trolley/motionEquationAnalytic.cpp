extern "C" {
#include <motionEquation.h>
}

#include <Eigen/Core>
#include <iostream>
#include <math.h>

extern "C" {

static double l;
static double m;
static double k;
static double d;
static double g;

void lib_init(double l, double m, double k, double d, double g) {
    ::l = l;
    ::m = m;
    ::k = k;
    ::d = d;
    ::g = -g;
}

void motionEquation(const double * x, double * xp) {
    Eigen::VectorXd dxdt(4);

    double t3 = x[2];
    double t2 = cos(t3);
    double t4 = x[3];
    double t5 = sin(t3);
    double t6 = t2 * t2;
    double t7 = l * l;
    double t8 = t4 * t4;
    dxdt << x[1],
        (d * t2 * t4 * 6.0 + k * t2 * t3 * 6.0 + m * t5 * t7 * t8 * 2.0 + g * l * m * t2 * t5 * 3.0) / (l * (m * 4.0 - m * t6 * 3.0)),
        t4,
        (1.0 / (l * l) * (d * t4 * 4.0 + k * t3 * 4.0 + g * l * m * t5 * 2.0 + m * t2 * t5 * t7 * t8) * 3.0) / (m * (t6 * 3.0 - 4.0));

    Eigen::Map< Eigen::VectorXd > xp_(xp, 4);
    xp_ = dxdt;
}

void kinematics(const double * x, double * pos, double * vel) {
    Eigen::Map< Eigen::VectorXd > pos_(pos, 4);
    pos_ << x[0], 0.0, x[0] + ::l * sin(x[2]), -::l * cos(x[2]);
    if (vel != nullptr) {
        Eigen::Map< Eigen::VectorXd > vel_(vel, 4);
        vel_ << x[1], 0.0, x[1] + ::l * cos(x[2]) * x[3], ::l * sin(x[2]) * x[3];
    }
}

void derivatives(const double * x, double * results) {
    const double q2 = x[2];
    const double dq2 = x[3];

    results[0] = -(6 * q2 * cos(q2)) / (l * m * (3 * cos(q2) * cos(q2) - 4));
    results[1] = (12 * q2) / (l * l * m * (3 * cos(q2) * cos(q2) - 4));
}

void lib_cleanup() {
}
}
