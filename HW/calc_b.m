function b = calc_b(in1,in2,in3)
%CALC_B
%    B = CALC_B(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    29-Oct-2021 08:52:22

c1 = in3(7,:);
c2 = in3(8,:);
dth1 = in1(3,:);
dth2 = in1(4,:);
g = in3(9,:);
l1 = in3(5,:);
m1 = in3(1,:);
m2 = in3(2,:);
tau1 = in2(1,:);
tau2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = l1.*t2;
t6 = cos(t4);
t7 = l1.*t3;
t8 = sin(t4);
t9 = c2.*t6;
t10 = c2.*t8;
t11 = dth1.*t9;
t12 = dth2.*t9;
t13 = dth1.*t10;
t14 = dth2.*t10;
t15 = t5+t9;
t16 = t7+t10;
t17 = dth1.*t15;
t18 = dth1.*t16;
t19 = t11+t12;
t20 = t13+t14;
t21 = t12+t17;
t22 = t14+t18;
t23 = t9.*t22.*2.0;
t24 = t10.*t21.*2.0;
t25 = -t24;
b = [tau1-(dth2.*m2.*(t23+t25-t15.*t20.*2.0+t16.*t19.*2.0))./2.0-g.*m2.*t16-c1.*g.*m1.*t3;tau2+(m2.*(t19.*t22.*2.0-t20.*t21.*2.0))./2.0-(dth2.*m2.*(t23+t25-t9.*t20.*2.0+t10.*t19.*2.0))./2.0-g.*m2.*t10];
