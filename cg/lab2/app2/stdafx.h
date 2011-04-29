// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#define _CRT_SECURE_NO_WARNINGS

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

// C RunTime Header Files
#include <tchar.h>

#pragma once


#include <cstdio>
#include <cassert>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>


#include "Geometry/primitives/point.h"
#include "Geometry/primitives/line.h"
#include "Geometry/primitives/color.h"
#include "Geometry/primitives/plane.h"
#include "Geometry/primitives/segment.h"
#include "common/PerfCounter.h"
#include "common/util.h"

#include "boost/optional.hpp"
#include "boost/shared_ptr.hpp"


using cg::point_3;
using cg::colorf;
using cg::colorb;
using cg::plane;
using cg::line_3;
using cg::segment_3;

using boost::optional;
using boost::shared_ptr;



// TODO: reference additional headers your program requires here
