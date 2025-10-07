#pragma once
#include <cmath>
#include <compare>
#include <iostream>
#include <tuple>

class Vector3D
{
    /// components of vector
   private:
    double X;
    double Y;
    double Z;

   public:
    Vector3D();
    Vector3D(const double& X1, const double& Y1, const double& Z1);
    Vector3D(const Vector3D& new_vector3d);

    /// @brief Getter
    /// @return geting vector component
    double GetX() const noexcept;
    double GetY() const noexcept;
    double GetZ() const noexcept;

    /// @brief Setter
    void SetX(double newX) noexcept;
    void SetY(double newY) noexcept;
    void SetZ(double newZ) noexcept;

    Vector3D operator+(const Vector3D& other) const noexcept;

    Vector3D operator-(const Vector3D& other) const noexcept;

    Vector3D operator*(double scalar) const noexcept;

    Vector3D operator/(double scalar) const;

    /// @brief calculat scalar product
    /// @param other other vector3d
    /// @return scalar product
    double Dot(const Vector3D& other) const noexcept;

    //// @brief calculat vector product
    /// @param other other vector3d
    /// @return vector3d
    Vector3D Cross(const Vector3D& other) const noexcept;

    /// @brief length of vector
    double LengthVector() const noexcept;

    /// @brief normilize vector
    Vector3D NormalizedVector() const;

    /// @brief distance between two points
    double DistanceToPoint(const Vector3D& p) const noexcept;

    auto operator<=>(const Vector3D&) const = default;

    /// @brief input and output function
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& p);

    friend std::istream& operator>>(std::istream& is, Vector3D& p);

    ~Vector3D() = default;
};