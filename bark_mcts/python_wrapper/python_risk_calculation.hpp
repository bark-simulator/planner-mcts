// Copyright (c) 2020 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_RISK_CALCULATION_HPP_
#define PYTHON_PYTHON_RISK_CALCULATION_HPP_

#include <pybind11/functional.h>
#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;

void python_risk_calculation(py::module m);

#endif  // PYTHON_PYTHON_RISK_CALCULATION_HPP_