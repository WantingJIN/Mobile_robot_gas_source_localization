fname = '../python/MyDeepCFD-master/Run/results.json'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);

% subplot(2,4,1);
% plot(val.curves.train_loss_curve);
% title('train loss curve');
% subplot(2,4,5);
% plot(val.curves.test_loss_curve);
% title('test loss curve');
% subplot(2,4,2);
% plot(val.curves.train_mse_curve);
% title('train mse curve');
% subplot(2,4,6);
% plot(val.curves.test_mse_curve);
% title('test mse curve');
% subplot(2,4,3);
% plot(val.curves.train_ux_curve);
% title('train ux curve');
% subplot(2,4,7);
% plot(val.curves.test_ux_curve);
% title('test ux curve');
% subplot(2,4,4);
% plot(val.curves.train_uy_curve);
% title('train uy curve');
% subplot(2,4,8);
% plot(val.curves.test_uy_curve);
% title('test uy curve');


% overlapped
subplot(2,4,1);
plot(val.curves.train_loss_curve);
hold on;
plot(val.curves.test_loss_curve);
title('loss curve');
legend('train', 'test');

subplot(2,4,2);
plot(val.curves.train_mse_curve);
hold on;
plot(val.curves.test_mse_curve);
title('mse curve');
legend('train', 'test');

subplot(2,4,3);
plot(val.curves.train_ux_curve);
hold on;
plot(val.curves.test_ux_curve);
xlabel('epochs');
title('ux curve');
legend('train', 'test');

subplot(2,4,4);
plot(val.curves.train_uy_curve);
hold on;
plot(val.curves.test_uy_curve);
title('uy curve');
legend('train', 'test');