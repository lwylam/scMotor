/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compile_lengths.cpp
 *
 * Code generation for function 'compile_lengths'
 *
 */

/* Include files */
#include "compile_lengths.h"
#include <cmath>

/* Function Definitions */
void compile_lengths(double in1[], double out1[]) // void compile_lengths(const double in1[6], double out1[4])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t9;
  double t10;
  double t11;
  double t12;
  double t61;
  double t16;
  double t25;
  double t26;
  double t66;
  double t28;
  double t29;
  double t30;
  double t35;
  double t36;
  double t37;
  double t38;
  double t39;
  double t40;
  double t57;
  double t65;
  double t50;
  double t51;
  double t59;
  double t60;
  double t62;
  double t63;
  double t67;
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double t89;
  double t90;
  double a_tmp;
  double b_a_tmp;

  /* COMPILE_LENGTHS */
  /*     OUT1 = COMPILE_LENGTHS(IN1,IN2,IN3,IN4) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     19-Feb-2020 17:34:30 */
  t2 = std::cos(in1[3]);
  t3 = std::cos(in1[4]);
  t4 = std::cos(in1[5]);
  t5 = std::sin(in1[3]);
  t6 = std::sin(in1[4]);
  t7 = std::sin(in1[5]);
  t9 = t2 * t4;
  t10 = t2 * t7;
  t11 = t4 * t5;
  t12 = t5 * t7;
  t61 = in1[0] * t3;
  t16 = t61 * t7;
  t25 = t10 + t6 * t11;
  t26 = t11 + t6 * t10;
  t66 = (in1[0] * t6 + in1[2] * t2 * t3) + -(in1[1] * t3 * t5);
  t28 = in1[2] * t26;
  t29 = t9 + -(t6 * t12);
  t30 = t12 + -(t6 * t9);
  t35 = t6 * (t66 + 0.5);
  t36 = t6 * (t66 - 0.5);
  t10 = t2 * t3;
  t37 = t10 * (t66 + 0.5);
  t11 = t3 * t5;
  t38 = t11 * (t66 + 0.5);
  t39 = t10 * (t66 - 0.5);
  t40 = t11 * (t66 - 0.5);
  t6 = in1[1] * t29;
  t9 = (t61 * t4 + in1[1] * t25) + in1[2] * t30;
  t12 = (-t16 + t28) + t6;
  t10 = (t16 - t28) - t6;
  t11 = t3 * t7;
  t57 = t11 * (t10 + 0.1);
  t5 = t11 * (t10 + 0.3);
  t65 = -t26 * (((t16 - t28) - t6) + 0.1);
  t66 = -t26 * (((t16 - t28) - t6) + 0.3);
  t10 = t3 * t4;
  t7 = t10 * (t9 + 0.5);
  t50 = t10 * (t9 - 0.5);
  t51 = t11 * (t12 + 0.1);
  t2 = t11 * (t12 + 0.3);
  t59 = t25 * (t9 + 0.5);
  t60 = t25 * (t9 - 0.5);
  t61 = t30 * (t9 + 0.5);
  t62 = t30 * (t9 - 0.5);
  t63 = t26 * (t12 + 0.1);
  t11 = t26 * (t12 + 0.3);
  t67 = t29 * (t12 + 0.1);
  t9 = t29 * (t12 + 0.3);
  a = t51 + t57;
  b_a = t2 + t5;
  c_a = t63 + t26 * (((t16 - t28) - t6) + 0.1);
  d_a = t11 + t26 * (((t16 - t28) - t6) + 0.3);
  t12 = t29 * (((t16 - t28) - t6) + 0.1);
  e_a = t67 + t12;
  t16 = t29 * (((t16 - t28) - t6) + 0.3);
  f_a = t9 + t16;
  t89 = std::sqrt((a * a + c_a * c_a) + e_a * e_a);
  t90 = std::sqrt((b_a * b_a + d_a * d_a) + f_a * f_a);
  a_tmp = t35 + t50;
  a = a_tmp + t5;
  b_a_tmp = t37 + t62;
  b_a = (b_a_tmp + t66) - 5.0;
  t10 = t38 - t60;
  c_a = (t10 + t16) + 0.1;
  d_a = a_tmp + -t2;
  e_a = (b_a_tmp + t11) - 5.0;
  f_a = (t10 + -t9) + 0.9;
  a_tmp = t35 + t7;
  t35 = (a_tmp + t5) - 5.0;
  b_a_tmp = t37 + t61;
  t29 = (b_a_tmp + t66) - 5.0;
  t10 = t38 - t59;
  t28 = (t10 + t16) + 0.1;
  t26 = (a_tmp + -t2) - 5.0;
  t25 = (b_a_tmp + t11) - 5.0;
  t30 = (t10 + -t9) + 0.9;
  t16 = t36 + t7;
  t4 = (t16 + t57) - 5.0;
  a_tmp = t39 + t61;
  t3 = a_tmp + t65;
  b_a_tmp = t40 - t59;
  t5 = (b_a_tmp + t12) + 0.1;
  t66 = (t16 + -t51) - 5.0;
  t61 = a_tmp + t63;
  t7 = (b_a_tmp + -t67) + 0.9;
  t16 = t36 + t50;
  t2 = t16 + t57;
  a_tmp = t39 + t62;
  t6 = a_tmp + t65;
  b_a_tmp = t40 - t60;
  t12 = (b_a_tmp + t12) + 0.1;
  t9 = t16 + -t51;
  t11 = a_tmp + t63;
  t10 = (b_a_tmp + -t67) + 0.9;
  out1[0] = (t90 + std::sqrt((a * a + b_a * b_a) + c_a * c_a)) + std::sqrt((d_a *
    d_a + e_a * e_a) + f_a * f_a);
  out1[1] = (t90 + std::sqrt((t35 * t35 + t29 * t29) + t28 * t28)) + std::sqrt
    ((t26 * t26 + t25 * t25) + t30 * t30);
  out1[2] = (t89 + std::sqrt((t4 * t4 + t3 * t3) + t5 * t5)) + std::sqrt((t66 *
    t66 + t61 * t61) + t7 * t7);
  out1[3] = (t89 + std::sqrt((t2 * t2 + t6 * t6) + t12 * t12)) + std::sqrt((t9 *
    t9 + t11 * t11) + t10 * t10);
}

/* End of code generation (compile_lengths.cpp) */
