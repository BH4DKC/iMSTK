/*=========================================================================

   Library: iMSTK

   Copyright (c) Kitware, Inc. & Center for Modeling, Simulation,
   & Imaging in Medicine, Rensselaer Polytechnic Institute.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0.txt

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=========================================================================*/

#pragma once

#include <iostream>

/// \todo remove nameless union/struct in the future
#pragma warning(disable : 4201)
namespace imstk
{
///
/// \struct Color
///
/// \brief Color in RGB space
///
struct Color
{
    union
    {
        double rgba[4];
        struct
        {
            double r;
            double g;
            double b;
            double a;
        };
    };

    ///
    /// \brief Constructor
    ///
    Color();
    Color(const double r, const double g, const double b, const double a = 1.0);
    Color(const double* rgba);

    ///
    /// \brief Constructor overwrites the alpha component
    ///
    Color(const Color color, const double a);

    ///
    /// \brief Equality operator
    ///
    Color& operator=(const Color& p_color);

    ///
    /// \brief Bitwise operator
    ///
    friend std::ostream& operator<<(std::ostream& os, const Color& c);

    ///
    /// \brief returns the color value given with the index
    ///
    double operator()(const int p_i) const;

    ///
    /// \brief Dark ratio. the value is between 0 and 1.0
    ///
    void darken(const double p_darkFactor);

    ///
    /// \brief lighten the color
    ///
    void lighten(const double p_darkFactor);

    ///
    /// \brief set RGB color
    ///
    void setValue(const double p_red,
                  const double p_green,
                  const double p_blue,
                  const double p_alpha = 1.0);

    ///
    /// \brief get RGB color
    ///
    void getValue(double color[4]);

    ///
    /// \brief get RGB color
    ///
    const double* getValue() const;

    ///
    /// \brief interpolate between two colors by ratio t
    ///
    static Color lerpRgba(const Color& start, const Color& end, const double t);
    static Color lerpRgb(const Color& start, const Color& end, const double t);

    /// Various commonly used colors
    static Color White;
    static Color Black;
    static Color DarkGray;
    static Color LightGray;
    static Color Blue;
    static Color Green;
    static Color Red;
    static Color Pink;
    static Color Orange;
    static Color Yellow;
};
#pragma warning(default : 4201)

///
/// \brief Multiplication operators
///
Color operator*(const Color& color_lhs, const Color& color_rhs);
Color operator*(const Color& color_lhs, const double intensity_rhs);
Color operator*=(const Color& color_lhs, const Color& color_rhs);
Color operator*=(const Color& color_lhs, const double intensity_rhs);

///
/// \brief Addition operators
///
Color operator+(const Color& color_lhs, const Color& color_rhs);
Color operator+(const Color& intensity_lhs, const double intensity_rhs);
Color operator+=(const Color& color_lhs, const Color& color_rhs);
Color operator+=(const Color& intensity_lhs, const double intensity_rhs);

///
/// \brief Subtraction operators
///
Color operator-(const Color& color_lhs, const Color& color_rhs);
Color operator-(const Color& color_rhs, const double intensity_lhs);
Color operator-=(const Color& color_lhs, const Color& color_rhs);
Color operator-=(const Color& color_rhs, const double intensity_lhs);
}
