function mu = single_t1(X,c)
    
    for i=1:size(X),  
        if(X[i]==c) mu = 1;
        else mu = 0;end
    end;

    end