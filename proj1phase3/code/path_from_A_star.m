function Optimal_path = path_from_A_star(map)
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;
    
    %start your code
    %% euclidean distance as cost function and herustic function
    function cost = dist(start_pos, goal_pos,dim)
        cost = sqrt(sum((double(start_pos) - double(goal_pos)).^2,dim));
    end
    G_score = inf(size(MAP));
    G_score(xStart, yStart)=0;
    herustic=zeros(size(MAP));

    [row,col]=ind2sub(size(G_score), 1:numel(G_score));
    dist_temp=dist([row;col],[xTarget;yTarget],1);
    herustic(1:end)=dist_temp(1:end);
    % unvisited=1, visted=inf
    unvisited = ones(size(MAP));
    backtrack = zeros(size(MAP,1),size(MAP,2),2);

    row_delta=[-1,-1,-1,0,0,1,1,1];
    col_delta=[-1,0,1,-1,1,-1,0,1];
    %row_delta=[-1,0,0,1];
    %col_delta=[0,1,-1,0];
    while(~all(unvisited==inf))
        % take lowest g(n) [masked]
       [min_val,idx]=min(unvisited(:).*(G_score(:)+herustic(:)));
       [row,col]=ind2sub(size(G_score),idx);
       % remove and mark as visited
       unvisited(row,col)=inf;
       % reach goal
       if row==xTarget && col==yTarget
           break;
       end

       for i=1:length(row_delta)
           temp_ind=[row+row_delta(i),col+col_delta(i)];
           %no obstacle, unvisted, within the index bound
           if all(temp_ind>0) && temp_ind(1) <= MAX_X && temp_ind(2) <=MAX_Y...
               && MAP(temp_ind(1),temp_ind(2))~=-1  && unvisited(temp_ind(1),temp_ind(2))==1

                temp = G_score(row,col)+dist([row,col],temp_ind,2);
                if temp < G_score(temp_ind(1),temp_ind(2))
                    G_score(temp_ind(1),temp_ind(2))=temp;
                    backtrack(temp_ind(1),temp_ind(2),:)=[row,col];
                end
           end 
       end
    end
    % no path found
    if G_score(xTarget,yTarget)==inf
        Optimal_path=[];
        disp("No path found");
    else
        ptr_ind = reshape(backtrack(xTarget,yTarget,:),[1,2]);
        Optimal_path=[xTarget,yTarget,map(1,3)];
        while (ptr_ind(1)~=xStart || ptr_ind(2)~=yStart)
            Optimal_path=[[ptr_ind,map(1,3)];Optimal_path];
            ptr_ind = reshape(backtrack(ptr_ind(1),ptr_ind(2),:),[1,2]);
        end
        Optimal_path=[[xStart,yStart,map(1,3)];Optimal_path];
        disp(Optimal_path)
        Optimal_path= Optimal_path-[0.5 0.5 0];
    end
    
end
