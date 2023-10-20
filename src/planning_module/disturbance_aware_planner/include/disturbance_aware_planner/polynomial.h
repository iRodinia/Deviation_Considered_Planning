#include <iostream>
#include <vector>
#include <cmath>

class Polynomial {
public:
    Polynomial(const std::vector<double>& coefficients) : coefficients_(coefficients) {}

    double evaluate(double x) const {
        double result = 0;
        for (size_t i = 0; i < coefficients_.size(); ++i) {
            result += coefficients_[i] * std::pow(x, i);
        }
        return result;
    }

    Polynomial derivative() const {
        std::vector<double> derivative_coefficients;
        for (size_t i = 1; i < coefficients_.size(); ++i) {
            derivative_coefficients.push_back(i * coefficients_[i]);
        }
        return Polynomial(derivative_coefficients);
    }

private:
    std::vector<double> coefficients_;
};

// polynomial转bspline函数实现：先将多项式转化为基函数系数向量，再构造BSpline对象。其中，基函数系数向量的长度为degree+2,第0个元素为常数项1,第degree+1个元素为常数项0。具体实现如下：
// BSpline polynomialToBSpline(const Polynomial& poly, const std::vector<double>& knots, int degree) {
//     int n = knots.size() + degree + 2;
//     n = std::min(n, static_cast<int>(knots.size() + degree + 2));
//     int m = poly.coefficients().size();
//     int maxOrder = m;
//     int nMax = n;
//     int order = degree + maxOrder;
//     int newOrder = std::min(order, nMax);
//     int newDegree = order - newOrder;
//     int newNMax = nMax + newOrder + newDegree + newDegree + degree + degree + degree + degree + degree + degree + degree;
//     nMax = newNMax;
//     n = std::min(nMax, static_cast<int>(knots.size() + degree + newDegree + newDegree + degree + degree + degree + degree + degree + degree));

//     // BSpline control points computation
//     std::vector<double> controlPoints(degree + newDegree + newDegree + degree + degree + degree + degree + degree + degree + degree, 0.0);
//     for (int i = 0; i <= newDegree; i++) {
//         for (int j = 0; j <= i; j++) {
//             double term1 = poly.coefficients()[m - i] * std::pow(knots[i + j], i);
//             double term2 = poly.coefficients()[m - j] * std::pow(knots[i + j], j);
//             controlPoints[j * (degree + newDegree) + i] += term1 * term2;
//         }
//     }

//     // BSpline basis functions computation
//     std::vector<std::vector<double>> basisFunctions(newOrder + 1, std::vector<double>(degree + newDegree));
//     for (int k = newOrder; k >= 0; k--) {
//         for (int l = 0; l <= k && l <= newDegree; l++) {
//             double term1 = std::pow(knots[l], k) * std::pow(knots[newDegree - l], newDegree);
//             double term2 = controlPoints[k * (degree + newDegree) + l];
//             basisFunctions[k][l] = term1 * term2;
//         }
//     }

//     return BSpline(basisFunctions, knots, n);
// }