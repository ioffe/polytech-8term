#pragma once
#include "macro.h"


// simple examples:
// ON_SCOPE_EXIT(&DestroyWindow, hwnd);
// or
// ON_SCOPE_EXIT_OBJ(*this, &this_type::log_message, L"done!")


namespace on_scope_exit_impl
{
   struct scope_guard_base
   {
      void disable() const
      {
         exec_ = false;
      }

   protected:
      scope_guard_base():
         exec_(true)
      {}

      scope_guard_base(scope_guard_base const& other):
         exec_(other.exec_)
      {
         other.disable();
      }

      template <typename T>
      static void exec_nothrow(T const& t)
      {
         if (t.exec_)
            try {t.exec();} catch (...) {}
      }

   private:
      mutable bool exec_;

   private:
      scope_guard_base const& operator = (scope_guard_base const&);
   };


   template <typename F>
   struct scope_guard_0: scope_guard_base
   {
      explicit scope_guard_0(F const& f):
         action_(f)
      {}

      void exec() const
      {
         action_();
      }

     ~scope_guard_0()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      F action_;
   };


   template <typename F, typename A1>
   struct scope_guard_1: scope_guard_base
   {
      scope_guard_1(F const& f, A1 const& a):
         action_(f),
         arg_(a)
      {}

      void exec() const
      {
         action_(arg_);
      }

     ~scope_guard_1()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      F        action_;
      A1 const arg_;
   };


   template <typename F, typename A1, typename A2>
   struct scope_guard_2: scope_guard_base
   {
      scope_guard_2(F const& f, A1 const& a1, A2 const& a2):
         action_(f),
         arg1_(a1),
         arg2_(a2)
      {}

      void exec() const
      {
         action_(arg1_, arg2_);
      }

     ~scope_guard_2()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      F        action_;
      A1 const arg1_;
      A2 const arg2_;
   };


   template <typename F, typename A1, typename A2, typename A3>
   struct scope_guard_3: scope_guard_base
   {
      scope_guard_3(F const& f, A1 const& a1, A2 const& a2, A3 const& a3):
         action_(f),
         arg1_(a1),
         arg2_(a2),
         arg3_(a3)
      {}

      void exec() const
      {
         action_(arg1_, arg2_, arg3_);
      }

     ~scope_guard_3()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      F        action_;
      A1 const arg1_;
      A2 const arg2_;
      A3 const arg3_;
   };


   template <typename O, typename MF>
   struct obj_scope_guard_0: scope_guard_base
   {
      obj_scope_guard_0(O& obj, MF mf):
         obj_(obj),
         action_(mf)
      {}

      void exec() const
      {
         CALL_MEMFUN(obj_, action_)();
      }

     ~obj_scope_guard_0()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      O& obj_;
      MF action_;
   };


   template <typename O, typename MF, typename A1>
   struct obj_scope_guard_1: scope_guard_base
   {
      obj_scope_guard_1(O& obj, MF mf, A1 const& a1):
         obj_(obj),
         action_(mf),
         arg_(a1)
      {}

      void exec() const
      {
         CALL_MEMFUN(obj_, action_)(arg_);
      }

     ~obj_scope_guard_1()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      O&       obj_;
      MF       action_;
      A1 const arg_;
   };

   template <typename O, typename MF, typename A1, typename A2>
   struct obj_scope_guard_2: scope_guard_base
   {
      obj_scope_guard_2(O& obj, MF mf, A1 const& a1, A2 const& a2):
         obj_(obj),
         action_(mf),
         arg1_(a1),
         arg2_(a2)
      {}

      void exec() const
      {
         CALL_MEMFUN(obj_, action_)(arg1_, arg2_);
      }

     ~obj_scope_guard_2()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      O&       obj_;
      MF       action_;
      A1 const arg1_;
      A2 const arg2_;
   };


   template <typename O, typename MF, typename A1, typename A2, typename A3>
   struct obj_scope_guard_3: scope_guard_base
   {
      obj_scope_guard_3(O& obj, MF mf, A1 const& a1, A2 const& a2, A3 const& a3):
         obj_(obj),
         action_(mf),
         arg1_(a1),
         arg2_(a2),
         arg3_(a3)
      {}

      void exec() const
      {
         CALL_MEMFUN(obj_, action_)(arg1_, arg2_, arg3_);
      }

     ~obj_scope_guard_3()
      {
         scope_guard_base::exec_nothrow(*this);
      }

   private:
      O&       obj_;
      MF       action_;
      A1 const arg1_;
      A2 const arg2_;
      A3 const arg3_;
   };

   // and so on
}


typedef on_scope_exit_impl::scope_guard_base const& scope_guard;


template <typename F>
inline on_scope_exit_impl::scope_guard_0<F>
make_scope_guard(F const& f)
{
   return on_scope_exit_impl::scope_guard_0<F>(f);
}

template <typename F, typename A1>
inline on_scope_exit_impl::scope_guard_1<F, A1>
make_scope_guard(F const& f, A1 const& a)
{
   return on_scope_exit_impl::scope_guard_1<F, A1>(f, a);
}

template <typename F, typename A1, typename A2>
inline on_scope_exit_impl::scope_guard_2<F, A1, A2>
make_scope_guard(F const& f, A1 const& a1, A2 const& a2)
{
   return on_scope_exit_impl::scope_guard_2<F, A1, A2>(f, a1, a2);
}

template <typename F, typename A1, typename A2, typename A3>
inline on_scope_exit_impl::scope_guard_3<F, A1, A2, A3>
make_scope_guard(F const& f, A1 const& a1, A2 const& a2, A3 const& a3)
{
   return on_scope_exit_impl::scope_guard_3<F, A1, A2, A3>(f, a1, a2, a3);
}


// todo: support const objects

template <typename O, typename MF>
inline on_scope_exit_impl::obj_scope_guard_0<O, MF>
make_obj_scope_guard(O& obj, MF mf)
{
   return on_scope_exit_impl::obj_scope_guard_0<O, MF>(obj, mf);
}

template <typename O, typename MF, typename A1>
inline on_scope_exit_impl::obj_scope_guard_1<O, MF, A1>
make_obj_scope_guard(O& obj, MF mf, A1 const& a1)
{
   return on_scope_exit_impl::obj_scope_guard_1<O, MF, A1>(obj, mf, a1);
}

template <typename O, typename MF, typename A1, typename A2>
inline on_scope_exit_impl::obj_scope_guard_2<O, MF, A1, A2>
make_obj_scope_guard(O& obj, MF mf, A1 const& a1, A2 const& a2)
{
   return on_scope_exit_impl::obj_scope_guard_2<O, MF, A1, A2>(obj, mf, a1, a2);
}

template <typename O, typename MF, typename A1, typename A2, typename A3>
inline on_scope_exit_impl::obj_scope_guard_3<O, MF, A1, A2, A3>
make_obj_scope_guard(O& obj, MF mf, A1 const& a1, A2 const& a2, A3 const& a3)
{
   return on_scope_exit_impl::obj_scope_guard_3<O, MF, A1, A2, A3>(obj, mf, a1, a2, a3);
}


#define ON_SCOPE_EXIT     ::scope_guard JOIN(___scope_guard_ref_, __COUNTER__) = ::make_scope_guard
#define ON_SCOPE_EXIT_OBJ ::scope_guard JOIN(___scope_guard_ref_, __COUNTER__) = ::make_obj_scope_guard
