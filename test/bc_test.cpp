/* Author: Hamal Marino
 * Desc:   Test for Bezier Curve 2D
 */

// C++
#include <string>
#include <vector>

#include <ros/ros.h>
#include <gtest/gtest.h>

// The header of the library to be tested
#include <bezier_curve/bezier_curve.h>

class BCTest
{
public:
    // A shared node handle
    // ros::NodeHandle nh_;
    
    // Ab object for the tests
    BezierCurve bc;
    
    // basic initialization test
    bool initialize()
    {
        BezierCurve::PointVector cp;
        cp.resize(3,4);
        cp << 0, 2,-1, 1,
              0, 1, 1, 0,
              0, 0, 0, 0;
        
        return bc.init_curve(cp);
    }
    
    bool testCoefficientVector(const std::string& id, const BezierCurve::CoefficientVector& expect, const BezierCurve::CoefficientVector& actual)
    {
        EXPECT_EQ(expect.size(), actual.size()) << id << " : Unequal vector sizes";
        
        static const double EPSILON = 0.000001;
        for (std::size_t i = 0; i < expect.size(); ++i)
        {
            EXPECT_GT(EPSILON, fabs(expect[i] - actual[i])) << "Section \'" << id << "\', Element " << i
            << ", Expect: " << expect[i] << ", Actual: " << actual[i];
        }
        
        return true;
    }
    
    // Try to use an uninitialized object. Succeeds if a std::runtime_error exception is thrown.
    bool testAllMethodException(const std::string& id, int ind)
    {
        try
        {
            if(ind == 0)
                bc.compute_point(0.5);
            else if(ind == 1)
                bc.compute_derivative(0.5);
            else if(ind == 2)
                bc.get_curve_coefficients();
            else if(ind == 3)
                bc.get_derivative_coefficients();
            else if(ind == 4)
                bc.get_order();
            else
            {
                ADD_FAILURE() << id << " : Unexpected index (" << ind << ")";
                return false;
            }
        }
        catch(std::runtime_error& e)
        {
            // std::cout << id << " : Caught exception : " << e.what() << std::endl;
            SUCCEED();
            return true;
        }
        catch(...)
        {
            ADD_FAILURE() << id << " : Unexpected exception";
            return false;
        }
        
        ADD_FAILURE() << id << " : Didn't throw exception as expected";
        return false;
    }
    
    // Try to use all object functions. Succeeds if no exception is thrown.
    bool testAllMethods(const std::string& id)
    {
        try
        {
            bc.compute_point(0.5);
            bc.compute_derivative(0.5);
            bc.get_curve_coefficients();
            bc.get_derivative_coefficients();
            bc.get_order();
        }
        catch(...)
        {
            ADD_FAILURE() << id << " : Unexpected exception";
            return false;
        }
        
        return true;
    }
};

/* Create instance of test class ---------------------------------------------------------- */
BCTest base;

/* Run tests ------------------------------------------------------------------------------ */
// Test initialization of the class
TEST(BCTest, initialize)
{
    ASSERT_TRUE(base.initialize());
}

// Test wrong size initialization
TEST(BCTest, wrongInitialize)
{
    BezierCurve::PointVector cp;
    cp.setRandom(cp.rows(),11);
    EXPECT_FALSE(base.bc.init_curve(cp));
    cp.setRandom(cp.rows(),2);
    EXPECT_FALSE(base.bc.init_curve(cp));
}

// Test calling methods before initialization
TEST(BCTest, exceptionTest)
{
    for(int ind=0; ind<5; ++ind)
        EXPECT_TRUE(base.testAllMethodException("Before initialization",ind));
    
    base.initialize();
    
    EXPECT_TRUE(base.testAllMethods("Initialized object"));
}

// Test the curve order and the computed coefficients
TEST(BCTest, orderAndCoefficients)
{
    BezierCurve::PointVector cp;
    cp.setRandom(cp.rows(),4);
    size_t order(cp.cols() - 1);
    
    base.bc.init_curve(cp);
    EXPECT_EQ(base.bc.get_order(),order) << "Order not respected: was " << base.bc.get_order() << " instead of " << order;
    
    BezierCurve::CoefficientVector expected_3rd(4);
    BezierCurve::CoefficientVector expected_2nd(3);
    expected_3rd << 1,3,3,1;
    expected_2nd << 1,2,1;
    
    EXPECT_TRUE(base.testCoefficientVector("3rd order",expected_3rd,base.bc.get_curve_coefficients()));
    EXPECT_TRUE(base.testCoefficientVector("3rd order derivative",expected_2nd,base.bc.get_derivative_coefficients()));
    
    // Test a 6-th order curve coefficients to be equal to the derivative coefficients of a 7-th order one
    cp.setRandom(cp.rows(),7);
    base.bc.init_curve(cp);
    BezierCurve::CoefficientVector c6(base.bc.get_curve_coefficients());
    cp.setRandom(cp.rows(),8);
    base.bc.init_curve(cp);
    BezierCurve::CoefficientVector dc7(base.bc.get_derivative_coefficients());
    EXPECT_TRUE(base.testCoefficientVector("6th order equals 7th order derivative",c6,dc7));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "bezier_curve_tests");
    
    // ROS_INFO_STREAM_NAMED("bezier_curve_tests","I'm about to run your tests...");
    
    return RUN_ALL_TESTS();
}

/*
 reminders:
 EXPECT_FALSE(#1);
 EXPECT_EQ(#1,#2);
 EXPECT_TRUE(#1 == NULL);
 EXPECT_GT(#1,#2);
 EXPECT_LT(#1,#2) << "Append here an output message for displaying errors in this call: " << my_number;
 */