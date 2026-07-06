#include "camodocal/gpl/EigenQuaternionParameterization.h"

namespace camodocal
{

bool
EigenQuaternionParameterization::Plus(const double* x,
                                      const double* delta,
                                      double* x_plus_delta) const
{
    return manifold_.Plus(x, delta, x_plus_delta);
}

bool
EigenQuaternionParameterization::PlusJacobian(const double* x,
                                              double* jacobian) const
{
    return manifold_.PlusJacobian(x, jacobian);
}

bool
EigenQuaternionParameterization::Minus(const double* y,
                                       const double* x,
                                       double* y_minus_x) const
{
    return manifold_.Minus(y, x, y_minus_x);
}

bool
EigenQuaternionParameterization::MinusJacobian(const double* x,
                                               double* jacobian) const
{
    return manifold_.MinusJacobian(x, jacobian);
}

}
