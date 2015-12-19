extern "C" {
#include <motionEquation.h>
}
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <vector>

#include <boost/numeric/odeint.hpp>

int main(int argc, char ** argv) {
    typedef std::vector< double > state_type;

    std::vector< state_type > x_vec;
    std::vector< double > times;

    struct push_back_state_and_time {
        std::vector< state_type > & m_states;
        std::vector< double > & m_times;

        push_back_state_and_time(std::vector< state_type > & states, std::vector< double > & times)
            : m_states(states)
            , m_times(times) {
        }

        void operator()(const state_type & x, double t) {
            m_states.push_back(x);
            m_times.push_back(t);
        }
    };

    double l = 1.0;
    double m = 5.0;
    double k = 10.0;
    double d = 0.6;
    double g = -9.81;

    lib_init(l, m, k, d, g);

    double x[4] = {-0.5, 0.0, 0.9 * M_PI, 0.0};
    double xp[4];

    Eigen::Map< Eigen::VectorXd > x_(x, 4);
    Eigen::Map< Eigen::VectorXd > xp_(xp, 4);

    state_type x_start(4);
    Eigen::Map< Eigen::VectorXd > x_start_(x_start.data(), x_start.size());
    x_start_ = x_;

    Eigen::VectorXd rand(4);

    std::ofstream file;
    file.open("exp_pendulum_on_trolley_data.csv");

    /// print csv header
    file << "time, x1, x2, x3, x4, xp1, xp2, xp3, xp4" << std::endl;

    double t_step = 0.001; // 0.01;
    double t_end = 5;

    typedef boost::numeric::odeint::runge_kutta4< state_type > error_stepper_type;
    //typedef boost::numeric::odeint::runge_kutta_fehlberg78<state_type> error_stepper_type;
    error_stepper_type stepper;
    //typedef boost::numeric::odeint::controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    //controlled_stepper_type controlled_stepper;

    std::function< void(const state_type &, state_type &, double)> fnk = [&](const state_type & xx, state_type & dxdt, double t) {
#if EIGEN_WORLD_VERSION >= 3
        x_ = Eigen::Map< const Eigen::VectorXd >(xx.data(), xx.size());
#else
        for (size_t i = 0; i < 4; ++i) {
            x_[i] = xx[i];
        }
#endif
        motionEquation(x, xp);
#if EIGEN_WORLD_VERSION >= 3
        Eigen::Map< Eigen::VectorXd > dxdt_(dxdt.data(), dxdt.size());
        dxdt_ = xp_;
#else
        for (size_t i = 0; i < 4; ++i) {
            dxdt[i] = xp_[i];
        }
#endif
    };
    boost::numeric::odeint::integrate_const(stepper, fnk, x_start, 0.0, t_end, t_step, push_back_state_and_time(x_vec, times));

    //boost::numeric::odeint::integrate_adaptive(controlled_stepper, fnk, x_start, 0.0, t_end, t_step, push_back_state_and_time( x_vec , times ));
    //boost::numeric::odeint::integrate_adaptive(boost::numeric::odeint::make_controlled< error_stepper_type >( 1.0e-10 , 1.0e-6 ), fnk, x_start, 0.0, t_end, t_step, push_back_state_and_time( x_vec , times ));

    std::cout << std::endl
              << std::endl;
    for (size_t i = 0; i < times.size(); ++i) {
        Eigen::Map< Eigen::VectorXd > x_(x_vec[i].data(), 4);
        file << times[i] << " " << x_.transpose();
        Eigen::VectorXd pos(4), vel(4);
        kinematics(x_vec[i].data(), pos.data(), vel.data());
        file << " " << pos.transpose();
        file << " " << vel.transpose();
#ifdef USE_ADOLC
        double jacobian[2];
        derivatives(x_vec[i].data(), jacobian);
        Eigen::Map< Eigen::VectorXd > jac_(jacobian, 2);
        file << " " << jac_.transpose();
#endif
        file << std::endl;
    }

    file.flush();
    file.close();

    lib_cleanup();

    return 0;
}
