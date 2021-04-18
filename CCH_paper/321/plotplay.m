
axe = 1:0.1:11;
data_lb = axe.^2;
data_ub = 1.5*axe.^2;


v = [axe' data_lb';axe' data_ub'];
%S.Vertices = [0 0;2 2;3 5;4 8;4 4;3 2;2 1;0 0];
S.Vertices = v;
[number_of_face,~] = size(S.Vertices);
S.Faces = [1:number_of_face/2,number_of_face:-1:number_of_face/2+1];
S.FaceColor = 'blue';
S.FaceAlpha = 0.2;
S.EdgeColor = 'black';
S.LineStyle = '--';
S.LineWidth = 0.5;
patch(S)
hold on
plot([1 2 3])