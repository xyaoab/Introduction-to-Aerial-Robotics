function Optimal_path = path_from_A_star_3d(map)
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    MAP(xval,yval,zval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    MAP(xval,yval,zval)=1;
    
    %start your code
    %% euclidean distance as cost function and herustic function
    function cost = dist(start_pos, goal_pos,dim)
        cost = sqrt(sum((double(start_pos) - double(goal_pos)).^2,dim));
    end
    G_score = inf(size(MAP));
    G_score(xStart, yStart,zStart)=0;
    herustic=zeros(size(MAP));
    for i = 1:MAX_X
        for j = 1:MAX_Y
            for k = 1:MAX_Z
                if MAP(i,j,k) == 2
                    herustic(i,j,k) = sqrt(sum(abs([xTarget yTarget zTarget] - [i j k]).^2));
                end
            end
        end
    end

    % unvisited=1, visted=inf
    unvisited = ones(size(MAP));
    backtrack = zeros(size(MAP,1),size(MAP,2),size(MAP,3),3);

%     row_delta=[-1*ones(1,9),zeros(1,8),ones(1,9)];
%     col_delta=[-1*ones(1,3),zeros(1,3),ones(1,3),-1*ones(1,3),zeros(1,2),ones(1,3),-1*ones(1,3),zeros(1,3),ones(1,3)];
%     pag_delta=[-1,0,1,-1,0,1,-1,0,1,...
%         -1,0,1,-1,1,-1,0,1,...
%         -1,0,1,-1,0,1,-1,0,1];
    row_delta=[-1,1,0,0,0,0];
    col_delta=[0,0,-1,1,0,0];
    pag_delta=[0,0,0,0,-1,1];
    while(~all(unvisited==inf))
        % take lowest g(n) [masked]
       [min_val,idx]=min(unvisited(:).*(G_score(:)+herustic(:)));
       [row,col,pag]=ind2sub(size(G_score),idx);
       % remove and mark as visited
       unvisited(row,col,pag)=inf;
       % reach goal
       if row==xTarget && col==yTarget && pag==zTarget
           break;
       end

       for i=1:length(row_delta)
           temp_ind=[row+row_delta(i),col+col_delta(i),pag+pag_delta(i)];
           %no obstacle, unvisted, within the index bound
           if all(temp_ind>0)...
              && temp_ind(1) <= MAX_X && temp_ind(2) <=MAX_Y && temp_ind(3) <=MAX_Z...
               && MAP(temp_ind(1),temp_ind(2),temp_ind(3))~=-1 ...
               && unvisited(temp_ind(1),temp_ind(2), temp_ind(3))==1
                temp = G_score(row,col,pag)+dist([row,col,pag],temp_ind,2);
                if temp < G_score(temp_ind(1),temp_ind(2),temp_ind(3))
                    G_score(temp_ind(1),temp_ind(2),temp_ind(3))=temp;
                    backtrack(temp_ind(1),temp_ind(2),temp_ind(3),:)=[row,col,pag];
                end

           end 
       end
    end
    % no path found
    if G_score(xTarget,yTarget,zTarget)==inf
        Optimal_path=[];
        disp("No path found");
    else
        ptr_ind = reshape(backtrack(xTarget,yTarget,zTarget,:),[1,3]);
        Optimal_path=[xTarget,yTarget,zTarget];
        while (ptr_ind(1)~=xStart || ptr_ind(2)~=yStart || ptr_ind(3)~=zStart)
            Optimal_path=[ptr_ind;Optimal_path];
            ptr_ind = reshape(backtrack(ptr_ind(1),ptr_ind(2),ptr_ind(3),:),[1,3]);
        end
        Optimal_path=[[xStart,yStart,zStart];Optimal_path];
        disp("Waypoints")
        disp(Optimal_path)

        Optimal_path= Optimal_path-[0.5 0.5 0.5];
    end
    
end
