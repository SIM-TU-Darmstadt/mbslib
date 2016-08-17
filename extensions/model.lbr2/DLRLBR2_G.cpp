/*
 * Copyright (C) 2016
 * Simulation, Systems Optimization and Robotics Group (SIM)
 * Technische Universitaet Darmstadt
 * Hochschulstr. 10
 * 64289 Darmstadt, Germany
 * www.sim.tu-darmstadt.de
 *
 * This file is part of the MBSlib.
 * All rights are reserved by the copyright holder.
 *
 * MBSlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation in version 3 of the License.
 *
 * The MBSlib is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MBSlib.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <model.lbr2/dlrlbr2.hpp>

mbslib::TVectorX DLRLBR2_G(const mbslib::TVectorX & q) {

    mbslib::TVectorX torque_g(7);
    mbslib::TVector3 g(0, 0, -9.81);

    const double gx = g(0);
    const double gy = g(1);
    const double gz = g(2);

    const double q1 = q(0);
    const double q2 = q(1);
    const double q3 = q(2);
    const double q4 = q(3);
    const double q5 = q(4);
    const double q6 = q(5);
    const double q7 = q(6);

    const double t4 = 3.141592653589793 * (1.0 / 2.0);
    const double t2 = q2 - t4;
    const double t3 = sin(t2);
    const double t5 = cos(t2);
    const double t6 = cos(q1);
    const double t7 = gx * t6;
    const double t8 = sin(q1);
    const double t9 = gy * t8;
    const double t10 = t7 + t9;
    const double t11 = q4 + t4;
    const double t12 = gz * t5;
    const double t13 = t3 * t10;
    const double t14 = t12 + t13;
    const double t15 = sin(q7);
    const double t16 = sin(q3);
    const double t17 = gz * t3;
    const double t23 = t5 * t10;
    const double t18 = t17 - t23;
    const double t19 = cos(q3);
    const double t20 = sin(q5);
    const double t21 = sin(t11);
    const double t22 = cos(t11);
    const double t24 = gx * t8;
    const double t25 = cos(q5);
    const double t26 = t14 * t21;
    const double t27 = t18 * t19;
    const double t33 = gy * t6;
    const double t34 = t24 - t33;
    const double t28 = t16 * t34;
    const double t29 = t27 + t28;
    const double t30 = t22 * t29;
    const double t31 = t26 + t30;
    const double t32 = t16 * t18;
    const double t35 = cos(q6);
    const double t36 = sin(q6);
    const double t39 = t19 * t34;
    const double t37 = t32 - t39;
    const double t38 = cos(q7);
    const double t40 = t16 * (t24 - t33);
    const double t41 = t27 + t40;
    const double t42 = t22 * t41;
    const double t43 = t26 + t42;
    const double t44 = t20 * t37;
    const double t46 = t25 * t43;
    const double t45 = t44 - t46;
    const double t47 = t14 * t22;
    const double t53 = t21 * t41;
    const double t48 = t47 - t53;
    const double t49 = t25 * t37;
    const double t50 = t20 * t43;
    const double t51 = t49 + t50;
    const double t52 = t35 * t45;
    const double t55 = t36 * t48;
    const double t54 = t52 - t55;
    const double t56 = t36 * t45 * 1.493181E-5;
    const double t57 = t15 * t51 * 7.83971514E-2;
    const double t58 = t38 * t54 * 7.83971514E-2;
    const double t59 = t35 * t48 * 1.493181E-5;
    const double t60 = t56 + t57 + t58 + t59;
    const double t61 = t36 * t45 * 8.587220999999999E-5;
    const double t62 = t15 * t54 * 7.83971514E-2;
    const double t63 = t35 * t48 * 8.587220999999999E-5;
    const double t85 = t38 * t51 * 7.83971514E-2;
    const double t64 = t61 + t62 + t63 - t85;
    const double t65 = t15 * t54 * 5.721E-1;
    const double t70 = t38 * t51 * 5.721E-1;
    const double t66 = t65 - t70;
    const double t67 = t36 * t45 * 3.4506;
    const double t68 = t35 * t48 * 3.4506;
    const double t69 = t35 * t45 * 2.8785;
    const double t71 = t15 * t66;
    const double t72 = t15 * t51 * 5.721E-1;
    const double t73 = t38 * t54 * 5.721E-1;
    const double t74 = t72 + t73;
    const double t75 = t38 * t74;
    const double t77 = t36 * t48 * 2.8785;
    const double t76 = t69 + t71 + t75 - t77;
    const double t78 = t38 * (t52 - t55) * 5.721E-1;
    const double t79 = t72 + t78;
    const double t80 = t67 + t68;
    const double t81 = t19 * t19;
    const double t82 = t81 * (1.3E1 / 5.0E1);
    const double t83 = t16 * t16;
    const double t84 = t83 * (1.3E1 / 5.0E1);
    const double t86 = t35 * t48 * 1.3217755365E-2;
    const double t87 = t25 * t37 * 3.252705E-2;
    const double t88 = t20 * t43 * 3.252705E-2;
    const double t89 = t35 * t45 * 1.3217755365E-2;
    const double t90 = t15 * t54 * 1.493181E-5;
    const double t91 = t25 * t37 * 5.3073783E-3;
    const double t92 = t20 * t43 * 5.3073783E-3;
    const double t105 = t38 * t51 * 1.493181E-5;
    const double t106 = t15 * t51 * 8.587220999999999E-5;
    const double t107 = t36 * t48 * 1.3217755365E-2;
    const double t108 = t38 * t54 * 8.587220999999999E-5;
    const double t93 = t89 + t90 + t91 + t92 - t105 - t106 - t107 - t108;
    const double t94 = t25 * t37 * 1.52050819;
    const double t95 = t20 * t43 * 1.52050819;
    const double t96 = t21 * t41 * 1.3537823E-2;
    const double t97 = t35 * t45 * 3.252705E-2;
    const double t98 = t14 * t22 * 1.5477585E-5;
    const double t99 = t38 * t60;
    const double t100 = t15 * t64;
    const double t101 = t38 * t79;
    const double t102 = t69 + t71 - t77 + t101;
    const double t103 = t36 * t80 * 3.385E-1;
    const double t104 = t20 * t37 * 5.4613594E-1;
    const double t109 = t36 * (t44 - t46) * 1.3217755365E-2;
    const double t110 = t15 * (t56 + t57 + t58 + t59);
    const double t122 = t38 * t64;
    const double t111 = t86 + t87 + t88 + t109 + t110 - t122;
    const double t112 = t82 + t84;
    const double t113 = t15 * t79;
    const double t114 = t25 * t37 * (4.07E2 / 1.0E2);
    const double t115 = t20 * t43 * (4.07E2 / 1.0E2);
    const double t140 = t38 * t66;
    const double t116 = t113 + t114 + t115 - t140;
    const double t117 = t35 * t102;
    const double t118 = t36 * t80;
    const double t119 = t20 * t37 * 1.1915;
    const double t120 = t82 + t84 - 1.489E-1;
    const double t121 = t14 * t21 * 9.0612805E-3;
    const double t123 = t25 * t37 * 1.5477585E-5;
    const double t124 = t20 * t43 * 1.5477585E-5;
    const double t125 = t22 * t41 * 9.0612805E-3;
    const double t126 = t20 * t37 * 1.3537823E-2;
    const double t127 = t19 * t34 * 5.30758445E-3;
    const double t176 = t16 * t18 * 5.30758445E-3;
    const double t177 = t35 * t93;
    const double t178 = t36 * t111;
    const double t179 = t25 * t43 * 1.3537823E-2;
    const double t128 = t121 + t123 + t124 + t125 + t126 + t127 - t176 - t177 - t178 - t179;
    const double t129 = t14 * t22 * 9.0612805E-3;
    const double t130 = t16 * t18 * 3.2495415E-2;
    const double t131 = t15 * t79 * 3.385E-1;
    const double t132 = t35 * t111;
    const double t158 = t14 * t22 * 1.3537823E-2;
    const double t159 = t36 * t93;
    const double t160 = t38 * t66 * 3.385E-1;
    const double t133 = t94 + t95 + t96 + t131 + t132 - t158 - t159 - t160;
    const double t134 = t25 * t133;
    const double t135 = t35 * t102 * 3.385E-1;
    const double t136 = t5 * t5;
    const double t137 = t136 * (1.0 / 4.0);
    const double t138 = t3 * t3;
    const double t139 = t138 * (1.0 / 4.0);
    const double t141 = t25 * t116;
    const double t147 = t25 * t43 * 1.1915;
    const double t148 = t117 + t118 + t119 - t147;
    const double t142 = t20 * t148;
    const double t143 = t14 * t22 * (2.13E2 / 5.0E1);
    const double t144 = t35 * t80;
    const double t145 = t20 * t116;
    const double t146 = t14 * t21 * 3.0685;
    const double t149 = t22 * t41 * 3.0685;
    const double t150 = t137 + t139 - 1.458E-1;
    const double t151 = t137 + t139;
    const double t165 = t25 * t148;
    const double t152 = t145 + t146 + t149 - t165;
    const double t153 = gz * t5 * 1.7877;
    const double t154 = t3 * t10 * 1.7877;
    const double t155 = t18 * t19 * 9.6696693E-3;
    const double t156 = t14 * t22 * 5.30758445E-3;
    const double t157 = t14 * t21 * 3.2495415E-2;
    const double t172 = t25 * t43 * 5.4613594E-1;
    const double t173 = t35 * t48 * 5.3073783E-3;
    const double t174 = t36 * t48 * 3.252705E-2;
    const double t175 = t21 * t41 * 1.5477585E-5;
    const double t161 = t97 + t98 + t99 + t100 + t103 + t104 + t135 - t172 - t173 - t174 - t175 - t36 * t45 * 5.3073783E-3;
    const double t162 = t35 * (t67 + t68);
    const double t163 = t143 + t162 - t21 * t41 * (2.13E2 / 5.0E1) - t36 * t102;
    const double t164 = t22 * t163;
    const double t166 = t21 * t152;
    const double t167 = t22 * t41 * 3.2495415E-2;
    const double t168 = t16 * (t24 - t33) * 9.6696693E-3;
    const double t169 = t153 + t154;
    const double t170 = t120 * t169;
    const double t171 = t16 * t18 * 9.6696693E-3;
    const double t180 = gz * t5 * 3.2822172E-3;
    const double t181 = t3 * t10 * 3.2822172E-3;
    const double t182 = t16 * t18 * 3.0685;
    const double t183 = t141 + t142 + t182 - t19 * t34 * 3.0685;
    const double t184 = t112 * t183;
    const double t185 = t16 * t18 * 1.7877;
    const double t186 = t185 - t19 * t34 * 1.7877;
    const double t187 = t120 * t186;
    const double t188 = t19 * (t24 - t33) * 5.30758445E-3;
    const double t189 = t121 + t123 + t124 + t125 + t126 - t176 - t177 - t178 - t179 + t188;
    const double t193 = t36 * (t44 - t46) * 5.3073783E-3;
    const double t194 = t97 + t98 + t99 + t100 + t103 + t104 + t135 - t172 - t173 - t174 - t175 - t193;
    const double t190 = t20 * t194;
    const double t191 = t129 + t130 + t134 + t190 - t19 * t34 * 3.2495415E-2 - t21 * t41 * 9.0612805E-3;
    const double t192 = t20 * t133;
    torque_g(0) = -t5 * (t184 + t187 + gz * t3 * 4.45105642E-3 - t5 * t10 * 4.45105642E-3 - t18 * t19 * 3.2822172E-3 - t16 * t34 * 3.2822172E-3 - t22 * t128 + t151 * (t19 * (t141 + t142 + t16 * t18 * 4.8562 - t19 * t34 * 4.8562) - t16 * (t18 * t19 * 1.7877 + t16 * t34 * 1.7877 + t22 * t152 - t21 * (t143 + t144 - t21 * t41 * (2.13E2 / 5.0E1) - t36 * t102))) - t150 * (gx * t8 * 5.0249 - gy * t6 * 5.0249) + t21 * (t129 + t130 + t134 - t19 * t34 * 3.2495415E-2 - t21 * t41 * 9.0612805E-3 + t20 * (t97 + t98 + t99 + t100 + t103 + t104 + t135 - t21 * t41 * 1.5477585E-5 - t25 * t43 * 5.4613594E-1 - t35 * t48 * 5.3073783E-3 - t36 * t48 * 3.252705E-2 - t36 * (t44 - t46) * 5.3073783E-3))) + t3 * (gx * t8 * (-1.173766391E-2) + gy * t6 * 1.173766391E-2 + gz * t5 * 4.45105642E-3 + t3 * t10 * 4.45105642E-3 - t19 * (t171 + t180 + t181 - t19 * t34 * 9.6696693E-3 + t21 * t128 + t22 * (t129 + t130 + t134 - t19 * t34 * 3.2495415E-2 - t21 * t41 * 9.0612805E-3 + t20 * t161)) + t16 * (t155 + t156 + t157 + t167 + t168 + t170 + t112 * (t21 * (t145 + t146 + t149 - t25 * (t117 + t118 + t119 - t25 * t43 * 1.1915)) + t22 * (t143 + t144 - t21 * t41 * (2.13E2 / 5.0E1) - t36 * t76)) - t21 * t41 * 5.30758445E-3 - t25 * (t97 + t98 + t99 + t100 + t103 + t104 - t21 * t41 * 1.5477585E-5 - t25 * t43 * 5.4613594E-1 - t36 * t45 * 5.3073783E-3 - t35 * t48 * 5.3073783E-3 - t36 * t48 * 3.252705E-2 + t35 * t76 * 3.385E-1) + t20 * (t94 + t95 + t96 + t15 * (t15 * (t20 * t31 + t25 * (t32 - t19 * (t24 - gy * t6))) * 5.721E-1 - t38 * (t35 * (t25 * t31 - t20 * t37) + t36 * (t47 - t21 * t29)) * 5.721E-1) * 3.385E-1 - t14 * t22 * 1.3537823E-2 - t38 * t66 * 3.385E-1 - t36 * t93 + t35 * (t86 + t87 + t88 + t15 * t60 + t36 * t45 * 1.3217755365E-2 - t38 * t64))));
    torque_g(1) = t151 * (t153 + t154 + t164 + t166) + t16 * (t171 + t180 + t181 - t19 * t34 * 9.6696693E-3 + t21 * t189 + t22 * t191) - gz * t3 * 1.173766391E-2 + t5 * t10 * 1.173766391E-2 + t150 * (gz * t5 * 5.0249 + t3 * t10 * 5.0249) + t19 * (t155 + t156 + t157 + t167 + t168 + t170 + t192 + t112 * (t164 + t166) - t21 * t41 * 5.30758445E-3 - t25 * t161);
    torque_g(2) = -t184 - t187 + t18 * t19 * 3.2822172E-3 + t22 * t189 - t21 * t191 + t16 * (t24 - t33) * 3.2822172E-3;
    torque_g(3) = t156 + t157 + t167 + t192 - t21 * t41 * 5.30758445E-3 - t25 * t194;
    torque_g(4) = t123 + t124 + t126 - t177 - t178 - t179;
    torque_g(5) = -t97 - t99 - t100 + t173 + t174 + t193;
    torque_g(6) = -t90 + t105 + t106 + t38 * (t52 - t55) * 8.587220999999999E-5;

    return torque_g;
}
