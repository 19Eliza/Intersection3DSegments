#include "vector3d.h"

Vector3D::Vector3D() : X(0.0), Y(0.0), Z(0.0) {};
Vector3D::Vector3D(const double& X1, const double& Y1, const double& Z1) : X(X1), Y(Y1), Z(Z1) {};
Vector3D::Vector3D(const Vector3D& new_vector3d)
{
    X = new_vector3d.X;
    Y = new_vector3d.Y;
    Z = new_vector3d.Z;
}

double Vector3D::GetX() const noexcept { return X; }
double Vector3D::GetY() const noexcept { return Y; }
double Vector3D::GetZ() const noexcept { return Z; }

void Vector3D::SetX(double newX) noexcept { X = newX; }
void Vector3D::SetY(double newY) noexcept { Y = newY; }
void Vector3D::SetZ(double newZ) noexcept { Z = newZ; }

Vector3D Vector3D::operator+(const Vector3D& other) const noexcept
{
    return {X + other.X, Y + other.Y, Z + other.Z};
}

Vector3D Vector3D::operator-(const Vector3D& other) const noexcept
{
    return {X - other.X, Y - other.Y, Z - other.Z};
}

Vector3D Vector3D::operator*(double scalar) const noexcept
{
    return {X * scalar, Y * scalar, Z * scalar};
}

Vector3D Vector3D::operator/(double scalar) const
{
    if (scalar == 0)
    {
        throw std::invalid_argument("Division vector (" + std::to_string(X) + ", " +
                                    std::to_string(Y) + ", " + std::to_string(Z) + ") by zero");
    }
    return {X / scalar, Y / scalar, Z / scalar};
}

double Vector3D::Dot(const Vector3D& other) const noexcept
{
    return X * other.X + Y * other.Y + Z * other.Z;
}

Vector3D Vector3D::Cross(const Vector3D& other) const noexcept
{
    return {Y * other.Z - Z * other.Y, Z * other.X - X * other.Z, X * other.Y - Y * other.X};
}

double Vector3D::LengthVector() const noexcept { return std::sqrt(X * X + Y * Y + Z * Z); }

Vector3D Vector3D::NormalizedVector() const
{
    double len = this->LengthVector();
    if (len == 0)
    {
        throw std::invalid_argument("Normalized vector with zero length.");
    }
    return {X / len, Y / len, Z / len};
}

double Vector3D::DistanceToPoint(const Vector3D& p) const noexcept
{
    double dx = X - p.X;
    double dy = Y - p.Y;
    double dz = Z - p.Z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::ostream& operator<<(std::ostream& os, const Vector3D& p)
{
    return os << "(" << p.X << ", " << p.Y << ", " << p.Z << ")";
}

std::istream& operator>>(std::istream& is, Vector3D& p) { return is >> p.X >> p.Y >> p.Z; }
