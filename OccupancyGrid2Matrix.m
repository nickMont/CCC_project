function Mat = OccupancyGrid2Matrix(OccGrid)

sizeGrid = OccGrid.GridSize;

Mat.bin = zeros(sizeGrid(1),sizeGrid(2));

for i = 1:sizeGrid(1)
    for j = 1:sizeGrid(2)
        Mat.bin(i,j) = ~getOccupancy(OccGrid,[i j],'grid');
    end
end

Mat.free = numel(find(Mat.bin == 0));
Mat.obs = numel(find(Mat.bin == 1));
Mat.workspace = sizeGrid(1)*sizeGrid(2);