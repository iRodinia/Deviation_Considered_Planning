#ifndef POLY3D
#define POLY3D

#include <vector>
#include <cmath>
#include <Eigen/Eigen>

class Polynomial3D{
    typedef Eigen::Matrix<double, 3, 1> Coef;
    typedef Eigen::Matrix<double, 3, -1> Coefs;
    typedef Eigen::Matrix<double, 3, 1> Value;

public:
    Polynomial3D(){
        order_ = 0;
        coefficients_ = Coef(0, 0, 0);
    }
    
    Polynomial3D(const Coefs& coefficients){
        if(coefficients.cols() <= 0){
            order_ = 0;
            coefficients_ = Coef(0, 0, 0);
        }
        else{
            order_ = coefficients.cols() - 1;
            coefficients_ = coefficients;
        }
    }

    void setCoefficients(const Coefs& coefficients){
        if(coefficients.cols() <= 0){
            order_ = 0;
            coefficients_ = Coef(0, 0, 0);
        }
        else{
            order_ = coefficients.cols() - 1;
            coefficients_ = coefficients;
        }
    }

    Coefs getCoefficients(){
        return coefficients_;
    }

    Value evaluate(double x) const {
        Value result = Value::Zero();
        for (size_t i = 0; i <= order_; ++i) {
            result += coefficients_.col(i) * std::pow(x, i);
        }
        return result;
    }

    Polynomial3D derivative() const {
        Coefs derivative_coefficients(3, order_);
        for (size_t i = 1; i <= order_; ++i) {
            derivative_coefficients.col(i-1) = i * coefficients_.col(i);
        }
        return Polynomial3D(derivative_coefficients);
    }

private:
    int order_;  // order = n
    Coefs coefficients_;   // [c0, c1, c2, ..., cn]
};

#endif