function STD=mean_sp(arrary)
STD=0;temp = [];
for tt=1:length(arrary)
    if arrary(tt) < 120
        temp(1,end+1) = arrary(tt);
    else
        continue;
    end
end
STD = std(temp);
end
