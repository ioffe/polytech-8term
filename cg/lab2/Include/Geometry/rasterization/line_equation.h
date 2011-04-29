#pragma once

#include "geometry\primitives\point.h"

namespace cg
{
   namespace line_eq
   {
    // неявное задание прямой L = {(x,y) | Ax + By + c = 0 }
    struct line_equation_2 {

       line_equation_2(point_2 const &P, point_2 const &Q)
          :   A_(Q.y - P.y)
          ,   B_(P.x - Q.x)
          ,   C_(-B_*P.y - A_*P.x)
       {}

       struct normalized {};


       line_equation_2& normalize() {
          double denom = cg::sqrt(A_*A_ + B_*B_);
          A_ /= denom;  B_ /= denom; C_ /= denom;
          return *this;
       }

       double operator () (point_2 const &R) const {
          return A_*R.x + B_*R.y + C_;
       }
    
        bool is_vertical() const {
            return eq(A_, 0.);
        }

        bool is_horizontal() const {
            return eq(B_, 0.);
        }

        bool undefined() const {
            return is_horizontal() && is_vertical();
        }

        // requires: !is_horizontal()
        double x_by_y(double y) const {
            return -(B_ * y + C_) / A_;
        }

        // requires: !is_vertical()
        double y_by_x(double x) const {
            return -(A_ * x + C_) / B_;
        }

        double x_step() const {
            return -B_ / A_;
        }

        double y_step() const {
            return -A_ / B_;
        }

    private:
       double A_,B_,C_;
    };
  }
}
