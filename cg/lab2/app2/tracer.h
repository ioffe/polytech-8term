#pragma once

#include "object.h"
#include "light.h"

struct tracer
{
   tracer();
   colorf trace(point_3 origin, point_3 dir);

private:
   colorf do_trace(point_3 origin, point_3 dir, double weight);
   void load_scene();

private:
   objects objs_;
   lights ls_;
   point_3 point_source_;
   cg::colorf ambient_;
};
