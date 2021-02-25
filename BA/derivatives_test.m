% this is just a formula to start with,  
% have fun and change it if you want to.  
f = @(x) x.^2 + 3*x - 1 + 5*x.*sin(x);  
t = @(x) x.^2;
% these next lines take the Anonymous function into a symbolic formula  
pkg load symbolic  
syms x;  
ff = f(x);  
% now calculate the derivative of the function  
ffd = diff(ff, x) 