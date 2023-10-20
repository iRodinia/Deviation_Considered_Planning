#include <iostream>
#include <vector>
#include <cmath>

class BSpline {
public:
    BSpline(const std::vector<double>& knots, const std::vector<double>& coefficients)
        : knots_(knots), coefficients_(coefficients) {}

    double evaluate(double x) const {
        return evaluateDerivative(x, 0);
    }

    double derivative(double x) const {
        return evaluateDerivative(x, 1);
    }

private:
    double evaluateDerivative(double x, int order) const {
        if (order >= coefficients_.size()) {
            return 0;
        }

        double result = 0;
        for (size_t i = 0; i < coefficients_.size(); ++i) {
            double term = coefficients_[i];
            for (int j = 0; j < order; ++j) {
                term *= (x - knots_[i + j]) / (knots_[i + order + j] - knots_[i + j]);
            }
            result += term;
        }

        return result;
    }

    std::vector<double> knots_;
    std::vector<double> coefficients_;
};