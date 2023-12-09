clear all; close all; clc;

pdf=load('/home/durancea/plume_model_learning/Results/cst_vel_comp_info_clust_m_comp/v0.1_d0/1/1701438110840/101/posterior_pdf_3D.csv');

x=linspace(2.75,12.75,20);
y=linspace(0,4,20);

for i=1:size(pdf,1)
    imagesc(x,y,reshape(pdf(i,:),20,20))
    pause(0.5);
end