#pragma once

struct intersect_detail
{
   point_3 pos;
   point_3 n;
   optional<double> t;
   optional<point_3> refract;
   optional<point_3> reflect;
};

struct material
{
   cg::colorf color;
   float ka;
   float kd;
   float kr;
   float ks, p;
   bool  fuzzy_refl;

   material();
};

struct object
{
   virtual bool intersect_ray(line_3 const & l, intersect_detail * detail) = 0;
   virtual bool intersect_seg(segment_3 const & l, intersect_detail * detail) = 0;
   virtual point_3 normal(point_3 const & p) = 0;

   material& mat();
   virtual ~object();

private:
   material mat_;
};

struct plane_obj : public object
{
   plane_obj(point_3 const & n, point_3 const & p);

   virtual bool intersect_ray(line_3 const & l, intersect_detail * detail);
   virtual bool intersect_seg(segment_3 const & l, intersect_detail * detail);
   virtual point_3 normal(point_3 const & p);

private:
   plane p_;
};

struct sphere_obj : public object
{
   sphere_obj(point_3 const & c, double r);

   virtual bool intersect_ray(line_3 const & line, intersect_detail * detail);
   virtual bool intersect_seg(segment_3 const & s, intersect_detail * detail);
   virtual point_3 normal(point_3 const & p);

private:
   optional<double> intersect_ray_impl(line_3 const & line);


private:
   point_3 center_;
   double radius_;
};

typedef boost::shared_ptr<object> object_ptr;
typedef std::vector<object_ptr> objects;
