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

namespace imstk
{
#define IMSTK_NOT_USED(x)

#define IMSTK_SET(name, dataType)           \
    virtual void set ## name(dataType _arg) \
    {                                       \
        if (this->m_ ## name != _arg)       \
        {                                   \
            this->m_ ## name = _arg;        \
        }                                   \
    }
#define IMSTK_GET(name, dataType) \
    virtual dataType get ## name() { return this->m_ ## name; }

// \todo Switch to template type lists
///
/// \brief Maps ScalarType type to templated function call
///
#define IMSTK_TYPE_CASE_STATEMENT(typeN, type, call) \
case typeN: { using IMSTK_TT = type; call; }; break

#define IMSTK_TYPE_CASE(call)                                              \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_CHAR, char, call);                     \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_UNSIGNED_CHAR, unsigned char, call);   \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_SHORT, short, call);                   \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_UNSIGNED_SHORT, unsigned short, call); \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_INT, int, call);                       \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_UNSIGNED_INT, unsigned int, call);     \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_LONG, long, call);                     \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_UNSIGNED_LONG, unsigned long, call);   \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_FLOAT, float, call);                   \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_DOUBLE, double, call);                 \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_LONG_LONG, long long, call);           \
    IMSTK_TYPE_CASE_STATEMENT(IMSTK_UNSIGNED_LONG_LONG, unsigned long long, call)

///
/// \brief Returns scalar type given template
///
#define IMSTK_TYPE_TEMPLATE(templateType)                                            \
    (std::is_same<templateType, char>::value ? IMSTK_CHAR :                          \
     (std::is_same<templateType, unsigned char>::value ? IMSTK_UNSIGNED_CHAR :       \
      (std::is_same<templateType, short>::value ? IMSTK_SHORT :                      \
       (std::is_same<templateType, unsigned short>::value ? IMSTK_UNSIGNED_SHORT :   \
        (std::is_same<templateType, int>::value ? IMSTK_INT :                        \
         (std::is_same<templateType, unsigned int>::value ? IMSTK_UNSIGNED_INT :     \
          (std::is_same<templateType, long>::value ? IMSTK_LONG :                    \
           (std::is_same<templateType, unsigned long>::value ? IMSTK_UNSIGNED_LONG : \
            (std::is_same<templateType, float>::value ? IMSTK_FLOAT :                \
             (std::is_same<templateType, double>::value ? IMSTK_DOUBLE :             \
              (std::is_same<templateType, long long>::value ? IMSTK_LONG_LONG :      \
               (std::is_same<templateType, unsigned long long>::value ? IMSTK_UNSIGNED_LONG_LONG : 0))))))))))))
}

// See https://www.fluentcpp.com/2019/08/30/how-to-disable-a-warning-in-cpp/
// When adding new warnings remember to add the DISABLE_ macro
// for all three sections MSVC, GCC/CLANG, other
#if defined(_MSC_VER)
#define IMSTK_DISABLE_WARNING_PUSH           __pragma(warning( push ))
#define IMSTK_DISABLE_WARNING_POP            __pragma(warning( pop ))
#define IMSTK_DISABLE_WARNING(warningNumber) __pragma(warning( disable : warningNumber ))

#define IMSTK_DISABLE_WARNING_UNREFERENCED_FORMAL_PARAMETER    IMSTK_DISABLE_WARNING(4100)
#define IMSTK_DISABLE_WARNING_UNREFERENCED_FUNCTION            IMSTK_DISABLE_WARNING(4505)
#define IMSTK_DISABLE_WARNING_HIDES_CLASS_MEMBER               IMSTK_DISABLE_WARNING(4458)
#define IMSTK_DISABLE_WARNING_PADDING                          IMSTK_DISABLE_WARNING(4324)
// other warnings you want to deactivate...

// Not seen in msvc or not checked, fix when working with windows

#elif defined(__GNUC__) || defined(__clang__)
#define DO_PRAGMA(X) _Pragma(#X)

#define IMSTK_DISABLE_WARNING_PUSH           DO_PRAGMA(GCC diagnostic push)
#define IMSTK_DISABLE_WARNING_POP            DO_PRAGMA(GCC diagnostic pop)
#define IMSTK_DISABLE_WARNING(warningName)   DO_PRAGMA(GCC diagnostic ignored #warningName)

#define IMSTK_DISABLE_WARNING_UNREFERENCED_FORMAL_PARAMETER    IMSTK_DISABLE_WARNING(-Wunused - parameter)
#define IMSTK_DISABLE_WARNING_UNREFERENCED_FUNCTION            IMSTK_DISABLE_WARNING(-Wunused - function)
// other warnings you want to deactivate...

// Not seen in gcc or not checked, fix when working with linux
#define IMSTK_DISABLE_WARNING_HIDES_CLASS_MEMBER
#define IMSTK_DISABLE_WARNING_PADDING

#else
#define IMSTK_DISABLE_WARNING_PUSH
#define IMSTK_DISABLE_WARNING_POP
#define IMSTK_DISABLE_WARNING_UNREFERENCED_FORMAL_PARAMETER
#define IMSTK_DISABLE_WARNING_UNREFERENCED_FUNCTION
#define IMSTK_DISABLE_WARNING_HIDES_CLASS_MEMBER
// other warnings you want to deactivate...

#endif