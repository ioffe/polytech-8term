#pragma once

namespace cg
{

// Throwed when infinite loop detected.
struct straight_skeleton_infinite_loop_exception
   : virtual std::exception, virtual boost::exception
{
   const char * what() const
   {
      return "Infinite loop detected in Straight Skeleton";
   }
};

}
