function A = calc_A(in1,in2)
%calc_A
%    A = calc_A(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    29-Oct-2021 08:52:22

I1 = in2(3,:);
I2 = in2(4,:);
c1 = in2(7,:);
c2 = in2(8,:);
l1 = in2(5,:);
m1 = in2(1,:);
m2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = c1.^2;
t6 = c2.^2;
t7 = l1.*t2;
t8 = cos(t4);
t9 = l1.*t3;
t10 = sin(t4);
t11 = c2.*t8;
t12 = c2.*t10;
t13 = t7+t11;
t14 = t9+t12;
t15 = t12.*t14.*2.0;
t16 = t11.*t13.*2.0;
t17 = t15+t16;
t18 = (m2.*t17)./2.0;
t19 = I2+t18;
A = reshape([I1+I2+(m1.*(t2.^2.*t5.*2.0+t3.^2.*t5.*2.0))./2.0+(m2.*(t13.^2.*2.0+t14.^2.*2.0))./2.0,t19,t19,I2+(m2.*(t6.*t8.^2.*2.0+t6.*t10.^2.*2.0))./2.0],[2,2]);
