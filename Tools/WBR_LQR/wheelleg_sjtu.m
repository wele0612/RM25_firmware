% =========== parameters ==========
syms Rw Rl Ll Lr Lwl Lwr Lbl Lbr Lc
syms Mw Ml Mb Iw Ill Ilr Ib Iz

syms g
% g = 9.81; % change this if not on earth

% state vector:
% [s ds phi dphi thll dthll thlr dthlr thb dthb]
% control input vector:
% [Twll Twlr Tbll Tblr]

% Direction: Tbl causes thb to decrease
% (pushes leg to the back)

% Direction: Twl causes thl to decrease
% Twl cause s(body velocity) to increase

% =========== Variables ==========
syms thwl thwr thll thlr thb phi s
syms ddthwl ddthwr ddthll ddthlr ddthb ddphi dds

syms sb hb sll slr hll hlr
syms Tlwl Tlwr Tbll Tblr fl fr 
syms Fwsl Fwsr Fwhl Fwhr Fbsl Fbsr Bbhl Fbhr

% =========== Equations ==========
eq1 = 0 == (Iw*Ll/Rw + Mw*Rw*Ll + Ml*Rw*Lbl)*ddthwl + ...
    (Ml*Lwl*Lbl - Ill)*ddthll + ...
    (Ml*Lwl + 1/2*Mb*Ll)*g*thll + Tbll - Tlwl*(1+Ll/Rw);

eq2 = 0 == (Iw*Lr/Rw + Mw*Rw*Lr + Ml*Rw*Lbr)*ddthwr + ...
    (Ml*Lwr*Lbr - Ilr)*ddthlr + ...
    (Ml*Lwr + 1/2*Mb*Lr)*g*thlr + Tblr - Tlwr*(1+Lr/Rw);

eq3 = 0 == -(Mw*Rw^2 + Iw + Ml*Rw^2 + 1/2*Mb*Rw^2)*ddthwl - ...
    (Mw*Rw^2 + Iw + Ml*Rw^2 + 1/2*Mb*Rw^2)*ddthwr - ...
    (Ml*Rw*Lwl + 1/2*Mb*Rw*Ll)*ddthll - ...
    (Ml*Rw*Lwr + 1/2*Mb*Rw*Lr)*ddthlr + ...
    Tlwl + Tlwr;

eq4 = 0 == (Mw*Rw*Lc + Iw*Lc/Rw + Ml*Rw*Lc)*ddthwl + ...
    (Mw*Rw*Lc + Iw*Lc/Rw + Ml*Rw*Lc)*ddthwr + ...
    Ml*Lwl*Lc*ddthll + Ml*Lwr*Lc*ddthlr - Ib*ddthb + ...
    Mb*g*Lc*thb - (Tlwl + Tlwr)*Lc/Rw - (Tbll + Tblr);
% original EQ4 from opensource document may have a typo!

eq5 = 0 == (1/2*Iz*Rw/Rl + Iw*Rl/Rw)*(ddthwl - ddthwr) + ...
    1/2*Iz*Ll/Rl*ddthll - 1/2*Iz*Lr/Rl*ddthlr + ...
    Rl/Rw*(-Tlwl + Tlwr);

wbr = solve([eq1,eq2,eq3,eq4,eq5],[ddthwl ddthwr ddthll ddthlr ddthb])
% disp(wbr.ddthll)
disp(simplify(wbr.ddthll))
disp(simplify(wbr.ddthlr))

% =========== Jacobian near equavelent (A) ==========

A = sym(zeros(10));

A(1,2) = 1;

A(2,5) = (Rw/2 * diff( wbr.ddthwl + wbr.ddthwr, thll));
A(2,7) = (Rw/2 * diff( wbr.ddthwl + wbr.ddthwr, thlr));
A(2,9) = (Rw/2 * diff( wbr.ddthwl + wbr.ddthwr, thb)); % = 0

A(3,4) = 1;

A(4,5) = ( ...
    Rw/(2*Rl) * diff( -wbr.ddthwl + wbr.ddthwr, thll) + ...
    diff( -wbr.ddthll, thll)*Ll/(2*Rl) + ...
    diff( wbr.ddthlr, thll)*Lr/(2*Rl));
A(4,7) = ( ...
    Rw/(2*Rl) * diff( -wbr.ddthwl + wbr.ddthwr, thlr) + ...
    diff( -wbr.ddthll, thlr)*Ll/(2*Rl) + ...
    diff( wbr.ddthlr, thlr)*Lr/(2*Rl));
A(4,9) = ( ...
    Rw/(2*Rl) * diff( -wbr.ddthwl + wbr.ddthwr, thb) + ...
    diff( -wbr.ddthll, thb)*Ll/(2*Rl) + ...
    diff( wbr.ddthlr, thb)*Lr/(2*Rl)); % = 0

A(5,6) = 1;

A(6,5) = diff(wbr.ddthll, thll);
A(6,7) = diff(wbr.ddthll, thlr);
A(6,9) = diff(wbr.ddthll, thb);

A(7,8) = 1;

A(8,5) = diff(wbr.ddthlr, thll);
A(8,7) = diff(wbr.ddthlr, thlr);
A(8,9) = diff(wbr.ddthlr, thb);

A(9,10) = 1;

A(10,5) = diff(wbr.ddthb, thll);
A(10,7) = diff(wbr.ddthb, thlr);
A(10,9) = diff(wbr.ddthb, thb);

% =========== Jacobian near equavelent (B) ==========
B = sym(zeros(10,4));

fprintf("Jacobian for B...\n");
B(2,1) = ( Rw/2 * diff( wbr.ddthwl + wbr.ddthwr, Tlwl));
B(2,2) = ( Rw/2 * diff( wbr.ddthwl + wbr.ddthwr, Tlwr));
B(2,3) = ( Rw/2 * diff( wbr.ddthwl + wbr.ddthwr, Tbll));
B(2,4) = ( Rw/2 * diff( wbr.ddthwl + wbr.ddthwr, Tblr));

B(4,1) = ( ...
    Rw/(2*Rl) * diff( -wbr.ddthwl + wbr.ddthwr, Tlwl) + ...
    diff( -wbr.ddthll, Tlwl)*Ll/(2*Rl) + ...
    diff( wbr.ddthlr, Tlwl)*Lr/(2*Rl));
B(4,2) = ( ...
    Rw/(2*Rl) * diff( -wbr.ddthwl + wbr.ddthwr, Tlwr) + ...
    diff( -wbr.ddthll, Tlwr)*Ll/(2*Rl) + ...
    diff( wbr.ddthlr, Tlwr)*Lr/(2*Rl));
B(4,3) = ( ...
    Rw/(2*Rl) * diff( -wbr.ddthwl + wbr.ddthwr, Tbll) + ...
    diff( -wbr.ddthll, Tbll)*Ll/(2*Rl) + ...
    diff( wbr.ddthlr, Tbll)*Lr/(2*Rl));
B(4,4) = ( ...
    Rw/(2*Rl) * diff( -wbr.ddthwl + wbr.ddthwr, Tblr) + ...
    diff( -wbr.ddthll, Tblr)*Ll/(2*Rl) + ...
    diff( wbr.ddthlr, Tblr)*Lr/(2*Rl));

B(6,1) = diff(wbr.ddthll, Tlwl);
B(6,2) = diff(wbr.ddthll, Tlwr);
B(6,3) = diff(wbr.ddthll, Tbll); 
B(6,4) = diff(wbr.ddthll, Tblr);

B(8,1) = (diff(wbr.ddthlr, Tlwl));
B(8,2) = (diff(wbr.ddthlr, Tlwr));
B(8,3) = (diff(wbr.ddthlr, Tbll)); 
B(8,4) = (diff(wbr.ddthlr, Tblr));

B(10,1) = (diff(wbr.ddthb, Tlwl));
B(10,2) = (diff(wbr.ddthb, Tlwr));
B(10,3) = (diff(wbr.ddthb, Tbll));
B(10,4) = (diff(wbr.ddthb, Tblr));

% =========== Numerical Solution ==========

Rwi = 0.06; % 驱动轮半径
Rli = 0.211; % 驱动轮轮距/2
Li = 0.22; % 腿长
% li = 0.2E3; % 腿长，毫米单位
Lwi = 160E-3; % 驱动轮质心到腿部质心距离

% Lbi = (0.8302*li - 94.083)*1E-3; % 机体质心到腿部质心距离
Lbi = 210E-3;

Lci = -0.05; % 机体质心到腿部关节距离
Mwi = 0.484; % 驱动轮质量
Mli = 0.943; % 腿部质量
Mbi = 10.1; % 机体质量
Iwi = 0.000943; % 驱动轮转动惯量
ILi = 0.0106;% 腿部转动惯量

Ibi = 0.138; % 机体pitch轴转动惯量
Izi = 0.248; % 机器人yaw轴总惯量

% 腿部质心到机体质心垂直距离
% y = 0.5632x - 30.538  X：腿长mm Y：垂直距离mm
% 腿部pitch（机体前后倾倒方向）转动惯量
% y = 0.1769x^2 + 42.89x + 23278 Y：转动惯量kg*mm^2  X：腿长mm

% Rwi = 0.09; % 驱动轮半径（米）
% Rli = 0.198625; % 驱动轮轮距/2（米）
% Li = 0.2; % 腿长（米）
% li = 0.2E3; % 腿长（毫米单位）
% 
% Lwi = (0.4667 * li + 28.408) * 1E-3; % 驱动轮质心到腿部质心距离（米）
% Lbi = (0.6976 * li - 48.418) * 1E-3; % 机体质心到腿部质心距离（米）
% Lci = 0.015; % 机体质心到腿部关节距离（米）
% 
% Mwi = 1.54; % 驱动轮质量（kg）
% Mli = 0.704; % 腿部质量（kg）
% Mbi = 15.578; % 机体质量（kg）
% 
% Iwi = 1258.524E-6; % 驱动轮转动惯量（kg·m²）
% ILi = (-0.0959 * li^2 + 46.542 * li + 9068.9) * 1E-6; % 腿部转动惯量（kg·m²）
% Ibi = 285049.635E-6; % 机体pitch轴转动惯量（kg·m²）
% Izi = (-0.4396 * li^2 + 128.52 * li + 489778) * 1E-6; % 机器人yaw轴总惯量（kg·m²）


geoparams = [Rw Rl Ll Lr        Lwl Lwr Lbl Lbr Lc];
geovalues = [Rwi Rli Li Li      Lwi Lwi Lbi Lbi Lci];
inerparams = [Mw Ml Mb  Iw      Ill Ilr Ib Iz];
inervalues = [Mwi Mli Mbi Iwi   ILi ILi Ibi Izi];

Av = double(subs(A, ...
    [geoparams inerparams g], [geovalues inervalues 9.81]))

Bv = double(subs(B, ...
    [geoparams inerparams g], [geovalues inervalues 9.81]))

Cv = eye(10)

Dv = zeros(10,4)

X_init = [0 0.1   0 0     -0.2 0     -0.2 0     0.1 0]

% =========== LQR controller ==========
% state vector:
% [s ds phi dphi thll dthll thlr dthlr thb dthb]
% control input vector:
% [Twll Twlr Tbll Tblr]

% approximatly works
% Q = diag([4 10000 10 1000 100 1 100 1 4000 1]);
% R = diag([200 200 100 100]);

% Q = diag([0.1 1000 800 100 500 1 500 1 50000 10]);
% R = diag([20 20 40 40]);

% Q = diag([400 4000 1000 2000 300 1 300 1 60000 1000]);
% R = diag([150 150 5 5]);

Q = diag([200 8000 400 2000 40 10 40 10 80000 1000]);
R = diag([300 300 20 20]);

Q2 = diag([400 4000 100 400 100 10 100 10 4000 1]);
R2 = diag([200 200 30 30]);

K = lqr(Av, Bv, Q, R)

K2 = lqr(Av, Bv, Q2, R2)

% 打印 K 矩阵
print_matrix_as_c_array(K, 'K_mat');

% 打印 K2 矩阵
print_matrix_as_c_array(K2, 'K2_mat');

% 定义一个函数来打印矩阵为 C 语言数组格式
function print_matrix_as_c_array(matrix, name)
    [rows, cols] = size(matrix);
    fprintf('const float %s[%d][%d] = {\n', name, rows, cols);
    for i = 1:rows
        fprintf('    {');
        for j = 1:cols
            if j < cols
                fprintf('%.5ff, ', matrix(i, j)); % 每行中间元素后加逗号
            else
                fprintf('%.5ff', matrix(i, j));   % 每行最后一个元素不加逗号
            end
        end
        if i < rows
            fprintf('},\n'); % 每行末尾加逗号，除了最后一行
        else
            fprintf('}\n');  % 最后一行结尾没有逗号
        end
    end
    fprintf('};\n\n');
end

