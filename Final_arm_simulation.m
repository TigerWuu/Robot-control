clc;
clear;

syms theta1 theta2 theta3 theta4 g1 g2 g3 g4;
l1=Link('d',5,'a',0,'alpha', pi/2,'m',0.055);
l2=Link('d',0,'a', 8,'alpha',0,'m',0.055);
l3=Link('d',0,'a', 8,'alpha', 0,'m',0.055);
l4=Link('d',0,'a', 6,'alpha', 0,'m',0.055);
 
EX = SerialLink([l1 l2 l3 l4], 'name', 'EX');

FKINE=EX.fkine([theta1 theta2 theta3 theta4])
EX.plot([0,0,0,0])


X=12;
Y=10;
Z=10;

[theta1, theta2 ,theta3 ,theta4]=Inverse(X,Y,Z ,-pi/3);
qf=[theta1, theta2 ,theta3 ,theta4];

%EX.plot([theta1 theta2 theta3 theta4]);
[p,pd,pdd]=mtraj(@lspb,[22 0 5],[X Y Z],50);       %position velocity acceleration
[q,qd,qdd]=jtraj(zeros(1,4),qf,50);                 %theta   omega   omega_dot
%% theta   omega   omega_dot-----t

subplot(3,1,1);plot(q);xlabel('Time (s)'); ylabel('\theta(rad)');legend('joint1','joint2','joint3','joint4');
subplot(3,1,2);plot(qd);xlabel('Time (s)'); ylabel('\omega(rad/s)');
subplot(3,1,3);plot(qdd);xlabel('Time (s)'); ylabel('\alpha(rad/s^{2})');

figure;
%% position velocity acceleration----t

subplot(3,1,1);plot(p);xlabel('Time (s)'); ylabel('p  (cm)');legend('x','y','z');
subplot(3,1,2);plot(pd);xlabel('Time (s)'); ylabel('v  (cm/s)');
subplot(3,1,3);plot(pdd);xlabel('Time (s)'); ylabel('a  (cm/s^{2})');

figure;
%% tau force -------t

force=pdd*0.055/100;
tau = EX.rne([q,qd,qdd]);
subplot(2,1,1),plot(force);xlabel('Time (s)'); ylabel('F(N)');legend('x','y','z');
subplot(2,1,2),plot(tau);xlabel('Time (s)'); ylabel('\tau  (kg*cm))');legend('joint1','joint2','joint3','joint4');

%% 
% 
% figure;
% 
% N1=0;
% N2=0;
% N3=0;
% m1=0.055;
% m2=0.055;
% m3=0.055;    %kg
% g=9.8;             % m/s^2
% 
% phi2=q(:,2);
% phi3=q(:,3);
% phi4=q(:,4);
% 
% omega2=qd(:,2);
% omega3=qd(:,3);
% omega4=qd(:,4);
% 
% aplha2=qdd(:,2);
% aplha3=qdd(:,3);
% aplha4=qdd(:,4);
% for i=1:50
%     R1=[cos(phi2(i,1)) -sin(phi2(i,1)) 0; sin(phi2(i,1)) cos(phi2(i,1)) 0; 0 0 1];  %% error
%     R2(i,1)=[cos(phi3(1,i)) -sin(phi3(i)) 0; sin(phi3(i)) cos(phi3(i)) 0; 0 0 1];
%     R3(i,1)=[cos(phi4(1,i)) -sin(phi4(i)) 0; sin(phi4(i)) cos(phi4(i)) 0; 0 0 1];
% end
% 
% 
% Pc1,P1=[l1;0;0];
% Pc2,P2=[l2;0;0];
% Pc3,P3=[l3;0;0];
% 
% F1=[-m1*l1*omega2^2+g*sin(phi2) ; m1*l1*alpha2+g*cos(phi2) ; 0];
% F2=[-m2*l2*omega3^2+g*sin(phi2+phi3) ; m2*l2*alpha3+g*cos(phi2+phi3) ; 0];
% F3=[-m3*l3*omega4^2+g*sin(phi2+phi3+phi4) ; m3*l3*alpha4+g*cos(phi2+phi3+phi4) ; 0];
% 
% f2=F3;
% f1=R2*f3+F2;
% 
% ni3=N3+cross(Pc3,F3);
% 
% n3=Idynamics(N3,R3,0,Pc3,F3,P3,0);
% ni2=n3;
% n2=Idynamics(N2,R2,ni2,Pc2,F2,P2,f2);
% ni1=n2;
% n1=Idynamics(N1,R1,ni1,Pc1,F1,P1,f1);
% 
% tau1=dot(n1,[0;0;1]);
% tau2=dot(n2,[0;0;1]);
% tau3=dot(n3,[0;0;1]);

figure;
%% move simulation
EX.plot(q);



%% function

function [theta1, theta2 ,theta3, theta4]=Inverse(X,Y,Z,the)
l1=5;
l2=9;
l3=9;
l4=5;
theta1=atan2(Y,X);

A=X-l4*cos(theta1)*cos(the);
B=Y-l4*sin(theta1)*cos(the);
C=Z-l1-l4*sin(the);
K=A^2+B^2+C^2-l2^2-l3^2;
theta3=-acos(K/2/l2/l3);

a=l3*sin(theta3);
b=l2+l3*cos(theta3);
c=Z-l1-l4*sin(the);
r=sqrt(a^2+b^2);
theta2=atan2(c,sqrt(r^2-c^2))-atan2(a,b);

theta4=the-theta2-theta3;

end

function n=Idynamics(N,R,ni,Pc,F,P,f)
n=N+R*ni+cross(Pc,F)+cross(P,R*f);
end



