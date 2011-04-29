#include "StdAfx.h"

#include "task.h"
#include "tracer.h"

static double _screen_pos_y  = 5.0;
static double _screen_max_x = 10.0;
static double _screen_max_z = 10.0;

static int _win_w;
static int _win_h;

static tracer tr;

static colorf trace(int scr_x, int scr_y)
{
   point_3 glb_pos(
      _screen_max_x * (((_win_w / 2) - scr_x) / (_win_w / 2.0)), 
      _screen_pos_y, 
      _screen_max_z * (((_win_h / 2) - scr_y)/ (_win_h/ 2.0)));

   point_3 glb_dir = cg::normalized_safe(glb_pos);
   //float color = (float)(glb_dir * point_3(0, 1, 0));
   //return colorf(color, color, color);

   return tr.trace(point_3(), glb_dir);
}

struct stored_pixel
{
   int x, y;
   colorf color;

   stored_pixel(int x = 0, int y = 0, colorf color = cg::color_black())
      : x(x), y(y), color(color)
   {}
};

const int stored_max = 1024;
static int stored_count = 0;
static stored_pixel pixels[stored_max];

void commit(HDC dc, HANDLE mutex)
{
      WaitForSingleObject(mutex, INFINITE);
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
      ReleaseMutex(mutex);
      stored_count = 0;
}

void store(int x, int y, colorf color, HDC dc, HANDLE mutex)
{
   if (stored_count + 1 >= stored_max)
      commit(dc, mutex);

   pixels[stored_count++] = stored_pixel(x, y, color);
}

void render(int win_width, int win_height, HDC dc, bool * alive, HANDLE mutex)
{
   _win_w = win_width;
   _win_h = win_height;
   BitBlt(dc, 0, 0, win_width, win_width, NULL, 0, 0, WHITENESS );
   for (int x = 0; x < win_width; ++x)
      for (int y = 0; y < win_height; ++y)
      {
         if (!(*alive))
            return;

         store(x, y, trace(x, y), dc, mutex);
      }
   if (*alive)
      commit(dc, mutex);
}

