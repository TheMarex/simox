#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <Eigen/Geometry>
#include <boost/format.hpp>

namespace VirtualRobot {
namespace Primitive {

class Primitive {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const int TYPE = 0;
    const int type;
    Eigen::Matrix4f transform;

    virtual std::string toXMLString(int tabs) = 0;

protected:
    Primitive(int type) : type(type), transform(Eigen::Matrix4f::Identity()) {}
    std::string getTransformString(int tabs = 0);
    std::string getXMLString(const std::string& type, const std::string& params, int tabs = 0);
private:
    Primitive() : type(TYPE) {}
};

class Box : public Primitive {
public:
    static const int TYPE = 1;
    Box() : Primitive(TYPE) {}
    float width;
    float height;
    float depth;
    std::string toXMLString(int tabs = 0);
};

class Sphere : public Primitive {
public:
    static const int TYPE = 2;
    Sphere() : Primitive(TYPE) {}
    float radius;
    std::string toXMLString(int tabs = 0);
};

typedef boost::shared_ptr<Primitive> PrimitivePtr;

} //namespace Primitive
} //namespace VirtualRobot

#endif // PRIMITIVE_H
