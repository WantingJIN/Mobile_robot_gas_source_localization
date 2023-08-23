close all; clc; clear all;

% calculation grid
X = 3:0.2:12.5;
Y = 0:0.2:4;
Z = 0.1;
Q= 500;
A = 0.005:0.001:0.01;
B = 0:0.1:0.5;
[X_grid,Y_grid,Z_grid] = meshgrid(X,Y,Z);

% source position
xs = 12.75;
ys = 2;
zs = 0.1;


k=1;
for sig_y_a=A
    for sig_y_b=B
        for z=Z
            for x=X
                for y=Y
                    adjusted_x = (xs-x);
                    if(adjusted_x >= 0)
%                         sigma_y = 0.000803 * adjusted_x*adjusted_x -0.004406 * adjusted_x + 0.1726;
%                         sigma_z = 0.0006798 * adjusted_x*adjusted_x -0.00244 * adjusted_x + 0.1562;
                        % simplified linear sigmas
                        sigma_y = sig_y_a * adjusted_x + sig_y_b;
                        sigma_z = 0.01 * adjusted_x + 0.1;
                        model_value(k) = (Q/(2*pi*1.2*sigma_y*sigma_z)) * exp( -(((ys-y)*(ys-y))/(2*sigma_y*sigma_y)) - (((zs-z)*(zs-z))/(2*sigma_z*sigma_z)));
                    else
                        model_value(k) = 0;
                    end
                    k = k+1;
                end
            end
        end
        % plot
        plot3(xs,ys,zs,'r+'); hold on;
        axis([0 20 0 4 0 2]); grid; colormap jet; colorbar; axis equal;
        scatter3(X_grid(:),Y_grid(:),Z_grid(:),5,model_value,'filled'); hold off;
        title(['sigy_a: ' num2str(sig_y_a) ', sigy_b: ' num2str(sig_y_b)]);
        load MyColormap.mat; colormap(mymap);
        k=1;
        pause();
    end
end