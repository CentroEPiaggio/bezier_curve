#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <eigen3/Eigen/Geometry>

class BezierCurve
{
public:
    typedef Eigen::Vector3d Point;
    typedef Eigen::Matrix3Xd PointVector;
    typedef Eigen::VectorXd CoefficientVector;

public:
    BezierCurve();
    ~BezierCurve();

    /**
     * @brief Initialize a curve giving a vector of control points (the order is inferred - at least 3 points are needed)
     * 
     * Takes a vector of points as input, and use them to initialize the curve. At least 3, and at most 10, points are needed.
     * 
     * @p cp vector of control points: has to contain between 3 and 10 points; on success, the passed vector is emptied.
     * @post on success, the parameter @cp is emptied
     * @return true on success
     */
    bool init_curve(PointVector& cp_);
    
    /**
     * @brief Compute a point on the parameterized curve.
     * 
     * @p t value between 0 and 1 to obtain a point on the curve
     * @return the point on the curve corresponding to t (or its saturation between 0 and 1)
     */
    Point compute_point(double t);
    
    /**
     * @brief Compute the derivative along the curve on a specific point.
     * 
     * @p t value between 0 and 1 to obtain a point on the curve
     * @return the derivative at the point on the curve corresponding to t (or its saturation between 0 and 1)
     */
    Point compute_derivative(double t);
    
    /**
     * @brief Returns the vector of coefficients used to compute the curve
     */
    CoefficientVector get_curve_coefficients();
    
    /**
     * @brief Returns the vector of coefficients used to compute the derivative
     */
    CoefficientVector get_derivative_coefficients();
    
    /**
     * @brief Return the order of the curve (equal to the number of control points minus 1)
     */
    int get_order();
    
private:
    /// the order of the curve
    int order;
    /// the vector of control points
    PointVector cp;
    /// the vector of coefficients to be used for computing the curve and its derivative
    CoefficientVector coeff,dcoeff;
    /// states whether the @init_curve function has been successfully called
    bool initialized;

private:
    /// a function to clamp @t between 0.0 and 1.0
    inline double clamp_t(const double& t) const { return t<0.0?0.0:(t>1.0?1.0:t); };
    
    /**
     * @brief Compute a Bezier curve of the appropriate order, as obtained during the call to @init_curve
     */
    Point compute_bezier_curve(const CoefficientVector& c,const PointVector& pts,double t) const;
    
    /**
     * @brief Compute the coefficient vector of an (@n-1)-th order Bezier curve. @n must be greater or equal than 2, lower values are considered to be equal to 2.
     */
    CoefficientVector compute_coefficients(int n);
    
    /// a utility function to throw an exception if trying to use a method of the class before initializing it
    inline void throw_if_not_initialized(std::string fcn);
};

#endif // BEZIER_CURVE_H
