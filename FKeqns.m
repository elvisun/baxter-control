%% DH
%  Assume that the s0 motor @ 0 is the origin/ground frame
as_v   = [0.0690,    0, 0.0690,    0, 0.0100,    0, 0.000]; 
as     = sym('a',[7 1]);
ds_v   = [0.2704,    0, 0.3644,    0, 0.3743,    0, 0.2295+0.14];
ds     = sym('d',[7 1]);
alphas = [ -pi/2, pi/2,  -pi/2, pi/2,  -pi/2, pi/2,     0];
% alphas = sym('al',[7 1]);
thetas = sym('T',[1 7]);
As = sym('As',[4,4,7]);
for i=1:7
    As(:,:,i) = homoMat(as(i), ds(i), alphas(i), thetas(i));
end
A04 = As(:,:,1);
for i=2:4
    A04 = A04*As(:,:,i);
end
A47 = As(:,:,5);
for i=6:7
    A47 = A47*As(:,:,i);
end
A04 = simplify(A04);
A47 = simplify(A47);
A07 = A04*A47;
A07 = subs(A07, [as(2) as(4) as(6) as(7) ds(2) ds(4) ds(6)], [0 0 0 0 0 0 0]);
A07 = simplify(A07);

