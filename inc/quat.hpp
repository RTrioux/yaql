#ifndef QUAT_HPP
#define QUAT_HPP
#include <iostream>
#include "vector3d.hpp"

class Quat
{
    public:
    Quat(double q0=0, double q1=0, double q2=0, double q3=0);
    Quat(double arr[4]);
    Quat(double q0,Vector3D im);
    virtual ~Quat(){}

    /** Tests **/
    bool isEqual(Quat const &) const;
    static bool isEqual(Quat const &, Quat const &);

    bool isNull() const;
    static bool isNull(Quat const &);

    /** Operators **/
    Quat operator+(Quat const &) const;
    Quat operator-(Quat const &) const;
    Quat operator-() const;
    Quat operator*(Quat const &) const;
    Quat operator/(Quat const &) const;
    bool operator==(Quat const &) const;
    bool operator!=(Quat const &) const;
    double & operator[](size_t index);
    double operator[](size_t index)const;
    friend std::ostream & operator<<(std::ostream &, Quat const &);

    
    /** Algebra **/
    Quat inverse() const;
    static Quat inverse(Quat const &);

    Quat conj() const;
    static Quat conj(Quat const &);

    Vector3D im() const;
    static Vector3D im(Quat const &);

    double re() const;
    static double re(Quat const &);

    // Norms
    double norm() const;
    static double norm(Quat const &);
    double norm2() const; // Norm squared
    static double norm2(Quat const &);

    Quat normalize() const;
    static Quat normalize(Quat const &);

    /** Rotations **/
    double getRotation() const;
    static double getRotation(Quat const &);

    // Unit quaternions
    //Quat unitQuat() const; // Convert the quaternion into a unit quaternion while keeping the rotation angle correct
    static Quat unitQuat(double angle, double x, double y, double z);
    static Quat unitQuat(double angle, Vector3D im);

    Vector3D rotateVector(Vector3D const &) const;
    static Vector3D rotateVector(Quat const &, Vector3D const &);

    /** Conversions **/
    //Quat fromEuler(double ypr[3]);
    //double * toEuler();

    /** Display **/
    void print() const;
    
    private:
    double m_arr[4];
    Vector3D m_im;
};



template<typename T> Quat operator/ (Quat const & q, T const & scalar)
{

    if(std::is_arithmetic<T>::value)
    {
        double arr[4];
        for (size_t i = 0; i < 4; i++)
        {
            arr[i] = q[i]/scalar;
        }
        return Quat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}

template<typename T> Quat operator/ (T const & scalar, Quat const & q)
{

    if(std::is_arithmetic<T>::value)
    {
        double arr[4];
        Quat q_inv = q.inverse();
        for (size_t i = 0; i < 4; i++)
        {
            arr[i] = scalar * q_inv[i];
        }
        return Quat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}



template<typename T> Quat operator* (Quat const & q, T const & scalar)
{
    if(std::is_arithmetic<T>::value)
    {
        double arr[4];
        for (size_t i = 0; i < 4; i++)
        {
            arr[i] = scalar * q[i];
        }
        return Quat(arr);
    }
    else
    {
        throw std::logic_error("Not a scalar");
    }
}

template<typename T> Quat operator* (T const & scalar, Quat const & q)
{
    return q*scalar;
}


#endif