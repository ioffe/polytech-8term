#pragma once

struct fuzzy_refl_calc
{
   fuzzy_refl_calc(point_3 const & v, point_3 const & n)
      : v(v)
      , n(n)
   {
   }

   double operator() (point_3 const & _l)
   {
      l = _l;
      h = cg::normalized_safe(l + v);

      double f = F();
      double d = D();
      double g = G();

      return d;
      return f * d * g / (n * l) / (n * v);
   }

private:
   double F()
   {
      return 1.;
      //return 1. / pow((1. + (h * l)), 5);
      //const double nu = 1.1;

      //double c = h * l;
      //double g = sqrt(nu * nu + c * c - 1);

      //return 
      //   0.5 * 
      //   ((c - g) / (c + g)) *
      //   (1 + cg::sqr((c * (c + g) - 1) / (c * (c - g) - 1)));
   }

   double D()
   {
      double a = cg::angle(n, h);
      const double m = .01;

      if (cg::abs(a) < cg::pi / 3)
         return 1;
      else
         return 0;

      return 
         1 / (4. * cg::pi * m*m * pow(n * h, 4.)) *
         exp(-cg::sqr(tan((a)) / m));
   }

   double G()
   {
      return cg::min(
         1.,
         2 * (n * h) * (n * v) / (v * h),
         2 * (n * h) * (n * l) / (v * h)
      );
   }

private:
   point_3 v;
   point_3 n;
   point_3 h;
   point_3 l;
};
