#include "bezier_curve/bezier_curve.h"
#include <iostream>

BezierCurve::BezierCurve() : order(-1),initialized(false)
{

}

BezierCurve::~BezierCurve()
{

}

bool BezierCurve::init_curve(BezierCurve::PointVector& cp_)
{
    cp.resize(Eigen::NoChange, 0);
    
    if (cp_.cols() < 3 || cp_.cols() > 10)
    {
        initialized = false;
        return false;
    }
    
    cp.swap<>(cp_);
    
    order = cp.cols()-1;
    coeff.resize(order+1);
    dcoeff.resize(order);
    
    dcoeff = compute_coefficients(order);
    coeff << 1,dcoeff.topRows(dcoeff.rows()-1) + dcoeff.bottomRows(dcoeff.rows()-1),1;
    
//     std::cout << "dcoeff: " << dcoeff << std::endl << std::endl;
//     std::cout << "coeff:" << coeff << std::endl << std::endl;
//     Eigen::VectorBlock<Eigen::VectorXd> c1(dcoeff.derived(),0,dcoeff.rows()-1);
//     Eigen::VectorBlock<Eigen::VectorXd> c2(dcoeff.derived(),1,dcoeff.rows()-1);
//     std::cout << "c1: " << c1 << std::endl << std::endl;
//     std::cout << "c2: " << c2 << std::endl << std::endl;
//     std::cout << dcoeff.topRows(dcoeff.rows()-1) + dcoeff.bottomRows(dcoeff.rows()-1) << std::endl;
    
    initialized = true;
    
    return true;
}

BezierCurve::Point BezierCurve::compute_point(double t)
{
    // base case: not initialized
    throw_if_not_initialized(__func__);
    
    return compute_bezier_curve(coeff,cp,t);
}

BezierCurve::Point BezierCurve::compute_derivative(double t)
{
    // base case: not initialized
    throw_if_not_initialized(__func__);
    
    // compute point differences, multiplied by the order
    PointVector new_cp;
    new_cp = order*(cp.rightCols(cp.cols()-1) - cp.leftCols(cp.cols()-1));
    
    return compute_bezier_curve(dcoeff,new_cp,t);
}

BezierCurve::Point BezierCurve::compute_bezier_curve(const BezierCurve::CoefficientVector& c, const BezierCurve::PointVector& pts, double t) const
{
    t = clamp_t(t);
    
    // base case t=0
    if(t == 0.0)
        return pts.leftCols(1);
    // base case t=1
    else if(t == 1.0)
        return pts.rightCols(1);
    
    double u = 1 - t;
    int order_here(pts.cols());
    
    // regular case: iterate among powers of t and u=(1-t)
    BezierCurve::CoefficientVector cc(c);
    double tn = std::pow(t,order_here-1);
    for(int i = 0; i<order_here; i++)
    {
        cc(i) *= tn;
        tn = tn/t*u;
    }
    return pts*cc;
}

BezierCurve::CoefficientVector BezierCurve::compute_coefficients(int n)
{
    BezierCurve::CoefficientVector coeff;
    
    // base case: the minimun considered order is 1 (n == 2)
    if (n < 3)
    {
        coeff.resize(2);
        coeff << 1,1;
    }
    else
    {
        coeff.resize(n);
        BezierCurve::CoefficientVector coeff_tmp(n-1);
        coeff_tmp = compute_coefficients(n-1);
        coeff << 1,coeff_tmp.topRows(coeff_tmp.rows()-1) + coeff_tmp.bottomRows(coeff_tmp.rows()-1),1;
    }
    
    return coeff;
}

int BezierCurve::get_order()
{
    // base case: not initialized
    throw_if_not_initialized(__func__);
    
    return order;
}

BezierCurve::CoefficientVector BezierCurve::get_curve_coefficients()
{
    // base case: not initialized
    throw_if_not_initialized(__func__);
    
    return coeff;
}

BezierCurve::CoefficientVector BezierCurve::get_derivative_coefficients()
{
    // base case: not initialized
    throw_if_not_initialized(__func__);
    
    return dcoeff;
}

void BezierCurve::throw_if_not_initialized(std::string fcn)
{
    if(!initialized)
    {
        std::string msg;
        msg.append(fcn).append(" : object not initialized!");
        std::runtime_error e(msg);
        throw e;
    }
}
