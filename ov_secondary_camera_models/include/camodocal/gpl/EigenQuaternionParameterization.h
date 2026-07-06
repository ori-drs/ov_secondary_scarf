#ifndef EIGENQUATERNIONPARAMETERIZATION_H
#define EIGENQUATERNIONPARAMETERIZATION_H

#include "ceres/manifold.h"

namespace camodocal
{

class EigenQuaternionParameterization : public ceres::Manifold
{
public:
    ~EigenQuaternionParameterization() override {}
    bool Plus(const double* x,
              const double* delta,
              double* x_plus_delta) const override;
    bool PlusJacobian(const double* x,
                      double* jacobian) const override;
    bool Minus(const double* y,
               const double* x,
               double* y_minus_x) const override;
    bool MinusJacobian(const double* x,
                       double* jacobian) const override;
    int AmbientSize() const override { return 4; }
    int TangentSize() const override { return 3; }

private:
    ceres::EigenQuaternionManifold manifold_;
};

}

#endif
