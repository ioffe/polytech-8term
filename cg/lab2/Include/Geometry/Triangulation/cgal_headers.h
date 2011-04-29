#pragma once

//#if defined(_SECURE_SCL) && _SECURE_SCL == 0
//#error "CGAL was compiled with checked iterators, and they are disabled in current project"
//#endif
//

#pragma warning( push )
#pragma warning( disable : 4311 4244 4996 4670 4673 )

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/intersection_2.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/noncopyable.hpp>

#pragma warning( pop )