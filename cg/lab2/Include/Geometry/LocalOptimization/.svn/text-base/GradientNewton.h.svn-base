// Numerical methods
namespace nm
{
   template< typename TEvaluator >
   float Minimize2D_GradientNewton( const TEvaluator &evalObj, const point_2f &x0,
      float lambda, float eps, point_2f *minPoint )
   {
      point_2f xiNext = x0;
      point_2f xi = xiNext;
      point_2f grad;
      static const int MAX_ITER_COUNT = 30;

      int curIter = 0;
      do
      {
         xi = xiNext;
         grad = evalObj.Gradient(xi);
         cg::matrix_2 Hessian = evalObj.Hessian(xi);
         cg::matrix_2 gradient2;
         cg::inverse(Hessian, gradient2);
         xiNext = xi - lambda * cg::normalized_safe(gradient2 * grad);
         curIter++;
      } while(norm(grad) > eps && curIter <= MAX_ITER_COUNT);

      if (minPoint != NULL)
         *minPoint = xi;
      float minVal = evalObj.Value(xi);

      return minVal;
   }
}
// END OF 'GRADIENTNEWTON.H' FILE
