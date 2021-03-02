function r = doubleInt(a,xmin,xmax,ymin,ymax,xne,yne)
%DOUBLEINT 此处显示有关此函数的摘要
%   此处显示详细说明
r = 0;
for xi = 1:xne
    x_left = xmin + (xmax-xmin)/xne*(xi-1);
    x_right = xmin + (xmax-xmin)/xne*xi;

    for yi = 1:yne
        y_left = ymin + (ymax-ymin)/yne*(yi-1);
        y_right = ymin + (ymax-ymin)/yne*yi;
        
        fval1 = a(x_left,y_left);
        fval2 = a(x_left,y_right);
        fval3 = a(x_right,y_left);
        fval4 = a(x_right,y_right);
        fval = 1/4*(fval1+fval2+fval3+fval4);
        r = r+ (ymax-ymin)/yne*(xmax-xmin)/xne*fval;
    end
end
end

