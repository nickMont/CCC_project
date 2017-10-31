clear;clc;

% map=[0 0 0
%     1 1 1];
% mapStart=[1;1]; mapEnd=[1;3];

% map=[0 0 0 0
%     1 0 1 0
%     1 0 1 0
%     1 0 0 0];
mapStart=[1;1;2]; mapEnd=[4;4;2];

mapFloor=[1 1 1 1
          1 1 0 1
          1 1 0 1
          1 1 1 1];
mapGame=[0 0 0 0
         1 1 0 0
         1 1 0 0
         1 1 0 0];
mapCeil=[1 1 1 1
         1 1 1 1
         1 1 1 1
         1 1 1 1];
map(:,:,1)=mapFloor;
map(:,:,2)=mapGame;
%map(:,:,2)=mapCeil;

[a,b,c]=size(map);
[c1,aVP1,~,~]=countPaths3D(map,zeros(size(map)),mapStart,mapEnd,0,[],{});
%[c2,aVP2,~,~]=countPaths(map(:,:,2),zeros(4,4),mapStart(1:2),[4,4],0,[],{});

%print solution pairs
% if c1<=25
%     for ii=1:c1
%         avpTemp=aVP1{ii}
%         %printmat=repmat('0',a,b);
%         printmat=zeros(a,b);
%         for ij=1:length(avpTemp)
%             printmat(avpTemp(1,ij),avpTemp(2,ij))=1;
%         end
%         printmat
%     end
% end

% %print the maze
% for ij=0:a+1
%     rowPrint='';
%     for ik=0:b+1
%         if (ij==0||ij==a+1)&&(ik==0||ik==b+1)
%             rowPrint=[rowPrint '+'];
%         elseif ij==0||ij==a+1
%             rowPrint=[rowPrint '-'];
%         elseif ik==0||ik==b+1
%             rowPrint=[rowPrint '|'];
%         else
%             if map(ij,ik)==0
%                 rowPrint=[rowPrint ' '];
%             else
%                 rowPrint=[rowPrint '#'];
%             end
%         end
%     end
%     fprintf(rowPrint);fprintf('\n');
% end












