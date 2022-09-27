function index_set = getTheBeginEndIndex(index_v)
    index_set = [];
   
    index_temp = find(index_v); 
    if( ~isempty(index_temp) )
    index_begin = index_temp(1);
    for o = 1:size(index_temp, 2)-1
        if(index_temp(o+1) - index_temp(o) > 10)
            index_end = index_temp(o);
            index_set = [index_set; [index_begin, index_end]];
            index_begin = index_temp(o+1);
        end
    end
    index_set = [index_set; [index_begin, index_temp(end)]];
    end
end

