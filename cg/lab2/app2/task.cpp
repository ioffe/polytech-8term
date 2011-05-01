#include "StdAfx.h"

#include "task.h"
#include "tracer.h"

static double _screen_pos_y  = 5.0;
static double _screen_max_x = 10.0;
static double _screen_max_z = 10.0;

static int _win_w;
static int _win_h;
static double _ratio;
static double _yfov;
static double _xfov;

static point_3 _xy;
static point_3 _ext_x;
static point_3 _ext_y;

static boost::scoped_ptr<tracer> tr;

const point_3 origin(0, -20, 0);
const cpr dir(0, 0, 0);
const double fov = 30;


static colorf trace(int scr_x, int scr_y)
{
   double x_pos_norm = ((double)scr_x) / (_win_w);
   double y_pos_norm = ((double)scr_y) / (_win_h);

   point_3 glb_dir = cg::normalized(_xy + x_pos_norm * _ext_x + y_pos_norm * _ext_y);

   return tr->trace(origin, glb_dir);
}

struct stored_pixel
{
   int x, y;
   colorf color;

   stored_pixel(int x = 0, int y = 0, colorf color = cg::color_black())
      : x(x), y(y), color(color)
   {}
};

const int stored_max = 1;
static int stored_count = 0;
static stored_pixel pixels[stored_max];

void commit(HDC dc, lock::critsec * cs)
{
      cs->lock();
         for (int i = 0; i < stored_count; ++i)
         {
            int x = pixels[i].x;
            int y = pixels[i].y;
            colorf color = colorf(
               cg::min(pixels[i].color.r, 1.f),
               cg::min(pixels[i].color.g, 1.f),
               cg::min(pixels[i].color.b, 1.f)
            );
            RECT r = {x, y, x + 1, y + 1};
            SetPixel(dc, x, y, RGB(color.r * 255, color.g * 255, color.b * 255));
         }
      cs->unlock();
      stored_count = 0;
}

void store(int x, int y, colorf color, HDC dc, lock::critsec * cs)
{
   if (stored_count + 1 >= stored_max)
      commit(dc, cs);

   pixels[stored_count++] = stored_pixel(x, y, color);
}

void render(int win_width, int win_height, HDC dc, bool * alive, lock::critsec * cs)
{
   tr.reset(new tracer);

   _win_w = win_width;
   _win_h = win_height;

   _ratio = (double)_win_h / _win_w;
   if (_ratio > 1)
   {
      _xfov = fov / 2;
      _yfov = _xfov * _ratio / 2;
   }
   else
   {
      _xfov = fov / _ratio / 2;
      _yfov = fov / 2;
   }

   _xy = polar_point_3( 1, -_xfov + dir.course, _yfov + dir.pitch);
   point_3 xY = polar_point_3( 1, -_xfov + dir.course, -_yfov + dir.pitch);
   point_3 Xy = polar_point_3( 1, _xfov + dir.course, +_yfov + dir.pitch);
   _ext_x = Xy - _xy;
   _ext_y = xY - _xy;
   double a = _ext_x * _ext_y;

   BitBlt(dc, 0, 0, win_width, win_width, NULL, 0, 0, WHITENESS );
   for (int x = 0; x < win_width; ++x)
      for (int y = 0; y < win_height; ++y)
      {
         if (!(*alive))
            return;

         store(x, y, trace(x, y), dc, cs);
      }
   if (*alive)
      commit(dc, cs);
}

