#include <iostream>
#include <math.h>
#include "quat.hpp"
using namespace std;


Quat::Quat(double q0,double q1, double q2, double q3)
{
    m_arr[0] = q0;
    m_arr[1] = q1;
    m_arr[2] = q2;
    m_arr[3] = q3;

    m_im = Vector3D(q1,q2,q3);
}

Quat::Quat(double arr[4])
{
    for (size_t i = 0; i < 4; i++)
    {
        m_arr[i] = arr[i];
    }
    m_im = Vector3D(arr[1],arr[2],arr[3]);
}

Quat::Quat(double q0, Vector3D im)
{
    m_arr[0] = q0;
    for (size_t i = 0; i < 3; i++)
    {
        m_arr[i+1] = im[i];
    }
    m_im = im;
}

/** Tests **/

bool Quat::isEqual(Quat const &q) const
{
    for (size_t i = 0; i < 4; i++)
    {
        if(m_arr[i] != q[i])
        {
            return false;
        }
    }
    return true;
}

bool Quat::isEqual(Quat const & q1, Quat const & q2)
{
    return q1.isEqual(q2);
}

bool Quat::isNull() const
{
    for (size_t i = 0; i < 4; i++)
    {
        if(m_arr[i] != 0)
        {
            return false;
        }
    }
    return true;
}

bool Quat::isNull(Quat const & q)
{
    return q.isNull();
}

/** Operators **/

Quat Quat::operator+(Quat const & q) const
{
    double arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = m_arr[i] + q[i];
    }
    return Quat(arr);
}

Quat Quat::operator-(Quat const & q) const
{
    double arr[4];
    for (size_t i = 0; i < 4; i++)
    {
        arr[i] = m_arr[i] - q[i];
    }
    return Quat(arr);
}

Quat Quat::operator-() const
{
    return Quat(-m_arr[0],-m_arr[1],-m_arr[2],-m_arr[3]);
}

Quat Quat::operator*(Quat const & q) const
{
    double q0 = m_arr[0] * q[0] - Vector3D::innerProd(m_im,q.m_im);
    Vector3D im = m_arr[0] * q.m_im + q[0] * m_im + Vector3D::crossProd(m_im,q.m_im);
    return Quat(q0, im);
}

Quat Quat::operator/(Quat const & q) const
{
    return (*this) * inverse(q);
}

bool Quat::operator!=(Quat const & q) const
{
    return !this->isEqual(q);
}

bool Quat::operator==(Quat const & q) const
{
    return this->isEqual(q);

}

double & Quat::operator[](size_t index)
{
    return m_arr[index];
}

double Quat::operator[](size_t index) const 
{
    return m_arr[index];
}

// friend operator
ostream & operator << (ostream &out, Quat const &q)
{
    out << "(";
    for (size_t i = 0; i < 4; i++)
    {
        out<<q.m_arr[i];
        if(i<3)
        {
            out<<",";
        }
    }
    out<<")";
    return out;
}


/** Algebra **/

Quat Quat::inverse() const
{
    return 1.0/norm2()*conj();
}

Quat Quat::inverse(Quat const & q)
{
    return q.inverse();
}

Quat Quat::conj() const 
{
    double arr[4]={m_arr[0]};
    for (size_t i = 1; i < 4; i++)
    {
        arr[i] = -m_arr[i];
    }
    return Quat(arr);
}

Quat Quat::conj(Quat const & q)
{
    return q.conj();
}

Vector3D Quat::im() const
{
    return m_im;
}

Vector3D Quat::im(Quat const & q)
{
    return q.im();
}

double Quat::re() const
{
    return m_arr[0];
}

double Quat::re(Quat const & q)
{
    return q.re();
}

// Norms

double Quat::norm2() const
{
    double norm2=0;
    for (size_t i = 0; i < 4; i++)
    {
        norm2+= (*this)[i] * (*this)[i];
    }
    return norm2;
}

double Quat::norm2(Quat const & q)
{
    return q.norm2();
}

double Quat::norm() const
{
    return sqrt(norm2());
}

double Quat::norm(Quat const & q)
{
    return q.norm();
}

Quat Quat::normalize() const
{
    return (*this) / norm();
}

Quat Quat::normalize(Quat const & q)
{
    return q.normalize();
}


/** Rotations **/

double Quat::getRotation() const
{
    return 2*acos(m_arr[0]);
}

double Quat::getRotation(Quat const & q)
{
    return q.getRotation();
}

// Unit quaternion

Quat Quat::unitQuat(double angle, double x, double y, double z)
{
    return unitQuat(angle, Vector3D(x,y,z));
}

Quat Quat::unitQuat(double angle, Vector3D im)
{
    if(angle == 0)
    {
        return Quat(1,0,0,0);
    }
    else
    {
        double q0 = cos(angle/2);
        if(angle<0)
        {
            im = -im;
        }
        double lambda = sqrt((1 - q0*q0)/im.norm2());
        return Quat(q0,lambda*im);
    }
}

Vector3D Quat::rotateVector(Vector3D const & vec) const
{
    Quat imQuat(0,vec[0],vec[1],vec[2]);
    imQuat = (*this) * imQuat *  inverse(*this);
    return Vector3D(imQuat[1],imQuat[2],imQuat[3]);
}

Vector3D Quat::rotateVector(Quat const & q, Vector3D const & vec)
{
    return q.rotateVector(vec);
}


/** Display **/

void Quat::print() const
{
    cout << "(";
    for (size_t i = 0; i < 4; i++)
    {
        cout<<m_arr[i];
        if(i<3)
        {
            cout<<",";
        }
    }
    cout<<")"<<endl;
}