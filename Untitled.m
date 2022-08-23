

for each = 1:length(cartis_obs)
    this_obs=cartis_obs(each);
    row=find(all_inform_obs(:,9) == this_obs);
    figure(each+1)
    plot(round(row/3),all_inform_obs(row,10)); hold on
    plot(round(row/3),all_inform_obs(row,1),'ro','MarkerSize',2);
    xlim([0 7435])
end

