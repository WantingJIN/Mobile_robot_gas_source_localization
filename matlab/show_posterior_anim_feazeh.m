clear all; close all; clc;

pdf=load('/data/FaezehRahbar/Projects/Data-driven_plume_model/Results/pdf.csv');

x=linspace(2.75,12.75,20);
y=linspace(0,4,20);

for i=40:size(pdf,1)
    imagesc(x,y,reshape(pdf(i,:),20,20))
    pause(0.5);
end